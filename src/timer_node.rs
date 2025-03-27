use rclrs::*;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use std_msgs::msg::String as StringMsg;

struct TimerNode {
    count: Arc<Mutex<i32>>,
    publisher: Arc<Publisher<StringMsg>>,
    node: Arc<Node>, // Keep the node alive
}

impl TimerNode {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("timer_node")?;
        let publisher = node.create_publisher::<StringMsg>("timer_topic")?;
        
        let timer_node = Arc::new(TimerNode {
            count: Arc::new(Mutex::new(0)),
            publisher,
            node,
        });
        
        // Create a background thread for timer functionality
        let timer_node_ref = Arc::clone(&timer_node);
        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_secs(1));
                timer_node_ref.timer_callback();
            }
        });
        
        Ok(timer_node)
    }
    
    fn timer_callback(&self) {
        let mut count = self.count.lock().unwrap();
        *count += 1;
        
        let mut msg = StringMsg::default();
        msg.data = format!("Timer triggered: count {}", *count);
        
        println!("Publishing: {}", msg.data);
        if let Err(e) = self.publisher.publish(&msg) {
            eprintln!("Error publishing: {}", e);
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _timer_node = TimerNode::new(&executor)?;
    
    println!("Timer node started. Publishing every second...");
    executor.spin(SpinOptions::default()).first_error()
}