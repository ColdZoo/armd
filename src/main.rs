//! armd 入口：加载配置、拉起各机械臂 worker、监听 Unix socket。

use armd::config::{ArmConfig, Config};
use armd::server::Daemon;
use std::sync::Arc;

fn usage() {
    println!(
        "armd {} — Dynamixel 机械臂串口守护进程\n\
\n\
用法: armd [--config <path>] [--socket <path>] [--sim] [--ids 1,2,3] [--rate 60]\n\
\n\
  --config <path>   JSON 配置（默认 ~/.config/armd/config.json）\n\
  --socket <path>   Unix socket 路径（覆盖配置）\n\
  --sim             无硬件自测模式（单臂，正弦轨迹）\n\
  --ids <list>      sim 模式的电机 id，默认 1,2,3,4,5,6\n\
  --rate <hz>       轮询频率，默认 60\n\
  --version         版本\n",
        env!("CARGO_PKG_VERSION")
    );
}

fn main() {
    let mut args = std::env::args().skip(1);
    let mut config_path: Option<String> = None;
    let mut socket: Option<String> = None;
    let mut sim = false;
    let mut ids: Vec<u8> = vec![1, 2, 3, 4, 5, 6];
    let mut rate = 60.0f64;

    while let Some(a) = args.next() {
        match a.as_str() {
            "--config" => config_path = args.next(),
            "--socket" => socket = args.next(),
            "--sim" => sim = true,
            "--ids" => {
                if let Some(v) = args.next() {
                    ids = v.split(',').filter_map(|s| s.trim().parse().ok()).collect();
                }
            }
            "--rate" => {
                if let Some(v) = args.next() {
                    rate = v.parse().unwrap_or(60.0);
                }
            }
            "--version" | "-V" => {
                println!("armd {}", env!("CARGO_PKG_VERSION"));
                return;
            }
            "--help" | "-h" => {
                usage();
                return;
            }
            other => {
                eprintln!("未知参数: {other}");
                usage();
                std::process::exit(2);
            }
        }
    }

    let cfg = if sim || config_path.is_none() {
        let path = config_path.clone().unwrap_or_else(default_config_path);
        if !sim {
            match Config::from_file(&path) {
                Ok(mut c) => {
                    if let Some(s) = socket.clone() {
                        c.socket = s;
                    }
                    c
                }
                Err(e) => {
                    eprintln!("[armd] 读取配置失败: {e}\n[armd] 可用 --sim 做无硬件自测");
                    std::process::exit(1);
                }
            }
        } else {
            Config {
                socket: socket.clone().unwrap_or_else(|| "/tmp/armd.sock".into()),
                listen: Vec::new(),
                token: None,
                identity: None,
                arms: vec![ArmConfig {
                    id: "sim".into(),
                    label: "sim arm".into(),
                    port: String::new(),
                    baud: 1_000_000,
                    poll_hz: rate,
                    read_timeout_ms: 30,
                    reconnect_sec: 2.0,
                    sim: true,
                    motors: ids,
                    poll_addr: None,
                    poll_len: 4,
                    monitor: false,
                }],
            }
        }
    } else {
        let path = config_path.unwrap();
        let mut c = match Config::from_file(&path) {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[armd] 读取配置失败: {e}");
                std::process::exit(1);
            }
        };
        if let Some(s) = socket {
            c.socket = s;
        }
        c
    };

    // 单实例：必须在打开串口前拿锁（避免两个 armd 同时驱动同一条总线）
    let lock_socket = cfg.listen_specs().iter()
        .find_map(|s| s.strip_prefix("unix:").map(|p| p.to_string()))
        .unwrap_or_else(|| cfg.socket.clone());
    let _instance_lock = match armd::server::acquire_instance_lock(&lock_socket) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("[armd] 启动失败: {e}");
            std::process::exit(1);
        }
    };
    let daemon = Arc::new(Daemon::new(&cfg));
    println!("[armd] {} 个机械臂: {:?}", cfg.arms.len(), daemon.arm_ids());
    if let Err(e) = daemon.serve() {
        eprintln!("[armd] 服务退出: {e}");
        std::process::exit(1);
    }
}

fn default_config_path() -> String {
    let home = std::env::var("HOME").unwrap_or_else(|_| "/tmp".into());
    format!("{home}/.config/armd/config.json")
}
