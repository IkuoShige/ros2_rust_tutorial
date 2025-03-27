use rclrs::*;
use example_interfaces::srv::AddTwoInts;
use std::sync::Arc;

struct AdditionServer {
    service: Arc<Service<AddTwoInts>>,
    node: Arc<Node>, // We need to keep the node alive
}

impl AdditionServer {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("addition_server")?;
        
        let service = node.create_service::<AddTwoInts, _>(
            "add_two_ints",
            |header, request| {  // Order is important: header first, then request
                println!("Received request: {} + {}", request.a, request.b);
                let sum = request.a + request.b;
                
                // Return the response directly, not wrapped in Ok()
                example_interfaces::srv::AddTwoInts_Response {
                    sum,
                }
            },
        )?;
        
        Ok(Arc::new(Self { 
            service,
            node, // Store the node to keep its lifetime
        }))
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _server = AdditionServer::new(&executor)?;
    
    println!("Service server ready. Waiting for requests...");
    
    // Spin the executor to process service requests
    executor.spin(SpinOptions::default()).first_error()
}