use rclrs::*;
use std::sync::{Arc, Mutex};
use std::{thread, time::Duration};

struct ParameterNode {
    // ノードへの参照を保持
    node: Arc<Node>,
    // 様々なタイプのパラメータを保持
    string_param: MandatoryParameter<Arc<str>>, // String -> Arc<str>
    int_param: MandatoryParameter<i64>,
    bool_param: MandatoryParameter<bool>,
    optional_param: OptionalParameter<Arc<str>>, // String -> Arc<str>
    read_only_param: ReadOnlyParameter<f64>,
    dynamic_param: MandatoryParameter<ParameterValue>,
    // パラメータが変更されたかを追跡
    param_changed: Mutex<bool>,
}

impl ParameterNode {
    fn new(executor: &Executor) -> Result<Arc<Self>, Box<dyn std::error::Error>> { // エラー型を変更
        // ノードを作成
        let node = executor.create_node("parameter_tutorial_node")?;
        
        // 1. 文字列パラメータ - 必須
        let string_param = node
            .declare_parameter("name")
            .default(Arc::from("ROS2 Rust Node")) // String -> Arc<str>
            .description("ノードの名前")
            .mandatory()?;
        
        // 2. 整数パラメータ - 範囲制約付き
        let int_range = ParameterRange {
            lower: Some(0),
            upper: Some(100),
            step: Some(10),
        };
        let int_param = node
            .declare_parameter("count")
            .default(50)
            .range(int_range)
            .description("カウント値 (0-100、10刻み)")
            .constraints("10の倍数である必要があります")
            .mandatory()?;
        
        // 3. ブールパラメータ
        let bool_param = node
            .declare_parameter("verbose")
            .default(false)
            .description("詳細なログ出力を有効にする")
            .mandatory()?;
        
        // 4. オプショナルパラメータ
        let optional_param = node
            .declare_parameter("message")
            .description("オプションのメッセージパラメータ")
            .optional()?;
        
        // 5. 読み取り専用パラメータ
        let read_only_param = node
            .declare_parameter("version")
            .default(1.0)
            .description("ノードのバージョン（変更不可）")
            .read_only()?;
        
        // 6. 動的型パラメータ
        let dynamic_param = node
            .declare_parameter("dynamic")
            .default(ParameterValue::String(Arc::from("初期値"))) // String -> Arc<str>
            .description("型が変更可能なパラメータ")
            .mandatory()?;
            
        // パラメータノードを構築
        let param_node = Arc::new(Self {
            node,
            string_param,
            int_param,
            bool_param,
            optional_param,
            read_only_param,
            dynamic_param,
            param_changed: Mutex::new(false),
        });
        
        // 初期パラメータ値を表示
        param_node.print_parameters();
        
        Ok(param_node)
    }
    
    // 現在のパラメータ値を表示
    fn print_parameters(&self) {
        println!("\n=== パラメータの現在値 ===");
        println!("name: {}", self.string_param.get());
        println!("count: {}", self.int_param.get());
        println!("verbose: {}", self.bool_param.get());
        match self.optional_param.get() {
            Some(value) => println!("message: {}", value),
            None => println!("message: [未設定]"),
        }
        println!("version: {}", self.read_only_param.get());
        println!("dynamic: {:?}", self.dynamic_param.get());
        println!("=========================\n");
    }
    
    // パラメータを更新
    fn update_parameters(&self) -> Result<(), Box<dyn std::error::Error>> { // エラー型を変更
        println!("\nパラメータを更新します...");
        
        // 各パラメータの変更を試みる
        self.string_param.set(Arc::from("更新された名前"))?; // String -> Arc<str>
        self.int_param.set(80)?;
        self.bool_param.set(true)?;
        
        // OptionalParameterには直接値を設定（Someでラップしない）
        self.optional_param.set(Arc::from("こんにちは ROS2 Rust!"))?;
        
        // 読み取り専用パラメータを変更しようとする (読み取り専用なのでsetメソッドはない)
        println!("読み取り専用パラメータは変更できません: {}", self.read_only_param.get());
        
        // 動的パラメータの型を変更
        self.dynamic_param.set(ParameterValue::Integer(42))?;
        
        // 変更フラグを設定
        *self.param_changed.lock().unwrap() = true;
        
        // 更新後のパラメータ値を表示
        self.print_parameters();
        Ok(())
    }
    
    // ノードの参照を取得
    fn get_node(&self) -> Arc<Node> {
        self.node.clone()
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> { // エラー型を変更
    // ROS2コンテキストを初期化
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    
    // パラメータノードを作成
    let param_node = ParameterNode::new(&executor)?;
    
    // ノードを実行
    println!("ノードを実行しています。以下のコマンドでパラメータを操作できます:");
    println!("  ros2 param list");
    println!("  ros2 param get /parameter_tutorial_node <param_name>");
    println!("  ros2 param set /parameter_tutorial_node <param_name> <value>");
    println!("  ros2 param describe /parameter_tutorial_node <param_name>");
    
    // 5秒後にパラメータを更新するスレッド
    let param_node_clone = param_node.clone();
    thread::spawn(move || {
        // 5秒待機
        thread::sleep(Duration::from_secs(5));
        
        // パラメータを更新
        if let Err(e) = param_node_clone.update_parameters() {
            eprintln!("パラメータの更新中にエラーが発生しました: {}", e);
        }
        
        println!("\nCtrl+Cで終了するまでノードを実行し続けます。");
    });
    
    // メインスレッドでスピン
    println!("ノードをスピンしています...");
    executor.spin(SpinOptions::default()).first_error().map_err(|e| e.into())
}