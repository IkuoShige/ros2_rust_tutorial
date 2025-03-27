use rclrs::*;
use std_msgs::msg::String as StringMsg;
use std::sync::{Arc, Mutex};

struct SubscriberNode {
    subscription: Mutex<Option<Arc<Subscription<StringMsg>>>>,
}

impl SubscriberNode {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("rust_subscriber")?;
        
        let subscriber_node = Arc::new(SubscriberNode {
            subscription: None.into(),
        });
        
        let subscription = node.create_subscription::<StringMsg, _>(
            "rust_topic",
            move |msg: StringMsg| {
                println!("I heard: '{}'", msg.data);
            },
        )?;
        
        *subscriber_node.subscription.lock().unwrap() = Some(subscription);
        
        Ok(subscriber_node)
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _subscriber = SubscriberNode::new(&executor)?;
    
    println!("Listening for messages on topic 'rust_topic'...");
    executor.spin(SpinOptions::default()).first_error()
}