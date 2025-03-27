use rclrs::*;
use std::sync::Arc;
use ros2_rust_tutorial_interfaces::srv::AddThreeInts;

struct AddThreeIntsServer {
    service: Arc<Service<AddThreeInts>>,
}

impl AddThreeIntsServer {
    fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("add_three_ints_server")?;
        
        let service = node.create_service::<AddThreeInts, _>(
            "add_three_ints",
            |request| {
                println!("Request: {} + {} + {}", request.a, request.b, request.c);
                let sum = request.a + request.b + request.c;
                
                let response = AddThreeInts::Response {
                    sum,
                };
                
                println!("Sending response: {}", sum);
                Ok(response)
            },
        )?;
        
        Ok(Arc::new(Self { service }))
    }
}

fn main() -> Result<(), RclrsError> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _server = AddThreeIntsServer::new(&executor)?;
    
    println!("Add Three Ints server is ready.");
    executor.spin(SpinOptions::default()).first_error()
}
