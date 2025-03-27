use rclrs::*;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;
use std_msgs::msg::Int32;
use std::{thread, time::Duration};

struct PubSubNode {
    subscription: Mutex<Option<Arc<Subscription<StringMsg>>>>,
    int_publisher: Arc<Publisher<Int32>>,
    message_count: Mutex<i32>,
}

impl PubSubNode {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("pubsub_node")?;
        
        // メッセージ数を公開するパブリッシャー
        let int_publisher = node.create_publisher::<Int32>("message_count")?;
        
        let pubsub_node = Arc::new(PubSubNode {
            subscription: None.into(),
            int_publisher,
            message_count: Mutex::new(0),
        });
        
        // サブスクリプションでメッセージを受け取る
        let pubsub_node_ref = Arc::clone(&pubsub_node);
        let subscription = node.create_subscription::<StringMsg, _>(
            "chatter",
            move |msg: StringMsg| {
                pubsub_node_ref.message_callback(msg);
            },
        )?;
        
        *pubsub_node.subscription.lock().unwrap() = Some(subscription);
        
        Ok(pubsub_node)
    }
    
    fn message_callback(&self, msg: StringMsg) {
        println!("Received: '{}'", msg.data);
        
        // カウンターをインクリメント
        let mut count = self.message_count.lock().unwrap();
        *count += 1;
        
        // 新しいカウントを公開
        let mut count_msg = Int32::default();
        count_msg.data = *count;
        
        println!("Publishing count: {}", count_msg.data);
        if let Err(e) = self.int_publisher.publish(&count_msg) {
            eprintln!("Error publishing count: {}", e);
        }
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    
    // PubSubノードを作成
    let _pubsub_node = PubSubNode::new(&executor)?;
    
    // 別ノードを作成して"chatter"トピックに発行
    let chatter_node = executor.create_node("chatter_publisher")?;
    let chatter_publisher = chatter_node.create_publisher::<StringMsg>("chatter")?;
    
    // 定期的にメッセージを発行するスレッド
    thread::spawn(move || -> Result<(), RclrsError> {
        let mut count = 0;
        loop {
            let mut msg = StringMsg::default();
            msg.data = format!("Hello chatter {}", count);
            
            println!("Publishing: {}", msg.data);
            chatter_publisher.publish(&msg)?;
            
            count += 1;
            thread::sleep(Duration::from_secs(1));
        }
    });
    
    println!("PubSub node is running...");
    executor.spin(SpinOptions::default()).first_error()
}
