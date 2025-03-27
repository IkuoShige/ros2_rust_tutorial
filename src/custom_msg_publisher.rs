use rclrs::*;
use std::{thread, time::Duration};
use ros2_rust_tutorial_interfaces::msg::Num;
use ros2_rust_tutorial_interfaces::msg::Sphere;
use geometry_msgs::msg::Point;

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("custom_msg_publisher")?;
    
    // 単純なカスタムメッセージのパブリッシャー
    let num_publisher = node.create_publisher::<Num>("topic_num")?;
    
    // 複雑なカスタムメッセージのパブリッシャー
    let sphere_publisher = node.create_publisher::<Sphere>("topic_sphere")?;
    
    thread::spawn(move || -> Result<(), RclrsError> {
        let mut count = 0;
        
        loop {
            // Numメッセージの発行
            let mut num_msg = Num::default();
            num_msg.num = count;
            println!("Publishing: Num({})", num_msg.num);
            num_publisher.publish(&num_msg)?;
            
            // Sphereメッセージの発行
            let mut sphere_msg = Sphere::default();
            sphere_msg.center = Point {
                x: (count as f64 * 0.1).sin(),
                y: (count as f64 * 0.1).cos(),
                z: 0.0,
            };
            sphere_msg.radius = 1.0 + (count as f64 * 0.05).sin() * 0.5;
            
            println!("Publishing: Sphere(center=[{:.2}, {:.2}, {:.2}], radius={:.2})",
                sphere_msg.center.x, sphere_msg.center.y, sphere_msg.center.z, sphere_msg.radius);
            sphere_publisher.publish(&sphere_msg)?;
            
            count += 1;
            thread::sleep(Duration::from_secs(1));
        }
    });
    
    println!("Custom message publisher started");
    executor.spin(SpinOptions::default()).first_error()
}