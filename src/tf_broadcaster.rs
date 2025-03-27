use rclrs::*;
use geometry_msgs::msg::TransformStamped;
use tf2_msgs::msg::TFMessage;
use builtin_interfaces::msg::Time as MsgTime;
use std::{thread, time::Duration};
use std::f64::consts::PI;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("tf_broadcaster")?;
    
    let tf_publisher = node.create_publisher::<TFMessage>("tf")?;
    
    // 時間サービスを取得
    let clock = node.get_clock();
    
    // 別スレッドでTF変換を定期的に発行
    thread::spawn(move || -> Result<(), RclrsError> {
        let mut time: f64 = 0.0;
        loop {
            // 現在の時刻を取得
            let now = clock.now();
            
            // 移動する変換を作成
            let mut transform = TransformStamped::default();
            
            // rclrs::Time から builtin_interfaces::msg::Time に変換
            let mut msg_time = MsgTime::default();
            
            // Time構造体から秒とナノ秒を取り出す
            // エラーメッセージによると、利用可能なフィールドは `nsec` と `clock` のみ
            msg_time.sec = (now.nsec / 1_000_000_000) as i32;  // ナノ秒から秒を計算
            msg_time.nanosec = (now.nsec % 1_000_000_000) as u32;  // 秒未満のナノ秒部分
            
            transform.header.stamp = msg_time;
            transform.header.frame_id = "world".to_string();
            transform.child_frame_id = "moving_frame".to_string();
            
            // 単純な円運動の軌跡
            transform.transform.translation.x = time.cos();
            transform.transform.translation.y = time.sin();
            transform.transform.translation.z = 0.0;
            
            // 単位四元数（回転なし）
            transform.transform.rotation.w = 1.0;
            transform.transform.rotation.x = 0.0;
            transform.transform.rotation.y = 0.0;
            transform.transform.rotation.z = 0.0;
            
            // TFメッセージを作成して発行
            let tf_message = TFMessage {
                transforms: vec![transform],
            };
            
            tf_publisher.publish(&tf_message)?;
            println!("Published transform: world -> moving_frame");
            
            // 時間を進める
            time += 0.1;
            if time > 2.0 * PI {
                time -= 2.0 * PI;
            }
            
            thread::sleep(Duration::from_millis(100));
        }
    });
    
    println!("TF broadcaster started. Press Ctrl+C to stop.");
    executor.spin(SpinOptions::default()).first_error()
}