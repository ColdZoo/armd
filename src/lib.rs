//! armd: 机械臂串口守护进程。
//!
//! 单进程、多机械臂：每个机械臂一个 worker 线程独占一条串口总线，
//! 对外只暴露 Unix socket 上的 RPC + 发布订阅（NDJSON）。

pub mod arm;
pub mod config;
pub mod dxl;
pub mod port;
pub mod profile;
pub mod server;
