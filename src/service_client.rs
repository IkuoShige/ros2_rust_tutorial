use rclrs::*;
use example_interfaces::srv::AddTwoInts;
use std::sync::{Arc, Mutex};
use std::{thread, time::Duration};

struct AdditionClient {
    client: Arc<Client<AddTwoInts>>,
    node: Arc<Node>,
    responses_received: Arc<Mutex<i32>>,
}

impl AdditionClient {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("addition_client")?;
        let client = node.create_client::<AddTwoInts>("add_two_ints")?;
        
        Ok(Arc::new(Self { 
            client, 
            node, 
            responses_received: Arc::new(Mutex::new(0)) 
        }))
    }
    
    fn call_service(&self, a: i64, b: i64) -> Result<(), RclrsError> {
        let request = example_interfaces::srv::AddTwoInts_Request { a, b };
        
        // サービスが利用可能になるまで待機
        while !self.client.service_is_ready()? {
            println!("Waiting for service...");
            thread::sleep(Duration::from_secs(1));
        }
        
        println!("Sending request: {} + {}", a, b);
        
        // Use callback pattern instead of manual polling
        let responses_counter = Arc::clone(&self.responses_received);
        self.client.async_send_request_with_callback(
            &request,
            move |response: example_interfaces::srv::AddTwoInts_Response| {
                println!("Received response: Sum = {}", response.sum);
                let mut counter = responses_counter.lock().unwrap();
                *counter += 1;
            },
        )?;
        
        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let client = AdditionClient::new(&executor)?;
    
    // 別スレッドでサービス呼び出し
    let client_thread = Arc::clone(&client);
    thread::spawn(move || {
        for i in 1..6 {
            if let Err(e) = client_thread.call_service(i, i * 10) {
                eprintln!("Error calling service: {}", e);
            }
            thread::sleep(Duration::from_secs(2));
        }
    });
    
    // Executor handles callbacks from service responses
    executor.spin(SpinOptions::default()).first_error()
}