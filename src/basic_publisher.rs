use rclrs::*;
use std_msgs::msg::String as StringMsg;
use std::{thread, time::Duration};

fn main() -> Result<(), RclrsError> {
    // 新しいAPIでContextとExecutorを作成
    let mut executor = Context::default_from_env()?.create_basic_executor();
    
    // ノードを作成
    let node = executor.create_node("rust_publisher")?;
    
    // パブリッシャーを作成
    let publisher = node.create_publisher::<StringMsg>("rust_topic")?;
    
    // パブリッシュスレッド
    thread::spawn(move || -> Result<(), RclrsError> {
        let mut count = 0;
        loop {
            let mut msg = StringMsg::default();
            msg.data = format!("Hello from Rust! Count: {}", count);
            println!("Publishing: {}", msg.data);
            publisher.publish(&msg)?;
            count += 1;
            thread::sleep(Duration::from_secs(1));
        }
    });
    
    // エグゼキューターを回す
    executor.spin(SpinOptions::default()).first_error()
}