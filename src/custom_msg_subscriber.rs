use rclrs::*;
use std::sync::{Arc, Mutex};
use ros2_rust_tutorial_interfaces::msg::Num;
use ros2_rust_tutorial_interfaces::msg::Sphere;

struct CustomMsgSubscriber {
    num_subscription: Mutex<Option<Arc<Subscription<Num>>>>,
    sphere_subscription: Mutex<Option<Arc<Subscription<Sphere>>>>,
}

impl CustomMsgSubscriber {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("custom_msg_subscriber")?;
        
        let subscriber = Arc::new(CustomMsgSubscriber {
            num_subscription: None.into(),
            sphere_subscription: None.into(),
        });
        
        // Numメッセージのサブスクリプション
        let num_subscription = node.create_subscription::<Num, _>(
            "topic_num",
            move |msg: Num| {
                println!("Received: Num({})", msg.num);
            },
        )?;
        
        // Sphereメッセージのサブスクリプション
        let sphere_subscription = node.create_subscription::<Sphere, _>(
            "topic_sphere",
            move |msg: Sphere| {
                println!("Received: Sphere(center=[{:.2}, {:.2}, {:.2}], radius={:.2})",
                    msg.center.x, msg.center.y, msg.center.z, msg.radius);
            },
        )?;
        
        *subscriber.num_subscription.lock().unwrap() = Some(num_subscription);
        *subscriber.sphere_subscription.lock().unwrap() = Some(sphere_subscription);
        
        Ok(subscriber)
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _subscriber = CustomMsgSubscriber::new(&executor)?;
    
    println!("Custom message subscriber started. Listening...");
    executor.spin(SpinOptions::default()).first_error()
}