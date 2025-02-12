use std::{env, error::Error, thread, time::Duration, sync::Arc};

use rclrs::{self, Context};
use geometry_msgs::msg::Twist;

use tokio::net::TcpListener;
use tokio_tungstenite::accept_async;
use futures_util::StreamExt; // for .next()

use serde::Deserialize;

/// Expected JSON payload: {"left": 0.5, "right": -0.8}
#[derive(Deserialize)]
struct TrackCommand {
    left: f64,
    right: f64,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    //--------------------------------------------------
    // 1) Initialize ROS (rclrs 0.4.x)
    //--------------------------------------------------
    let context = Context::new(env::args())?;
    // create_node(...) returns Arc<Node>, so no extra Arc needed
    let node = rclrs::create_node(&context, "motor_proxy")?;
    // create_publisher(...) returns Arc<Publisher<Twist>>
    let publisher = node.create_publisher::<Twist>("/cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;

    //--------------------------------------------------
    // 2) Spin the node in a separate thread
    //--------------------------------------------------
    let node_for_thread = node.clone();
    thread::spawn(move || {
        loop {
            // spin_once requires Arc<Node>
            if let Err(e) = rclrs::spin_once(node_for_thread.clone(), Some(Duration::from_millis(50)))
            {
                eprintln!("spin_once error: {:?}", e);
            }
        }
    });

    //--------------------------------------------------
    // 3) WebSocket server on 0.0.0.0:8080
    //--------------------------------------------------
    let ws_addr = "0.0.0.0:8081";
    let listener = TcpListener::bind(ws_addr).await?;
    println!("motor_proxy: listening on {}", ws_addr);

    // Accept connections forever
    loop {
        let (stream, addr) = listener.accept().await?;
        println!("New client: {}", addr);

        // Clone the Arc<Publisher<Twist>> for this client's task
        let pub_clone = publisher.clone();

        tokio::spawn(async move {
            if let Err(e) = handle_client(stream, pub_clone).await {
                eprintln!("Client {} error: {}", addr, e);
            }
        });
    }
}

/// Handle a single WebSocket client
async fn handle_client(
    stream: tokio::net::TcpStream,
    publisher: Arc<rclrs::Publisher<Twist>>,
) -> Result<(), Box<dyn Error>> {
    // Upgrade to WebSocket
    let mut ws_stream = accept_async(stream).await?;
    println!("Handshake completed.");

    // Continuously read text messages
    while let Some(msg_result) = ws_stream.next().await {
        let msg = msg_result?;
        if msg.is_text() {
            let text = msg.into_text()?;
            // Parse JSON: {"left": 0.5, "right": -0.8}
            let cmd: TrackCommand = match serde_json::from_str(&text) {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("JSON parse error: {}", e);
                    continue;
                }
            };
            println!("Received left={} right={}", cmd.left, cmd.right);

            // Convert to Twist
            let twist = tracks_to_twist(cmd.left, cmd.right);

            // Publish to /cmd_vel
            if let Err(e) = publisher.publish(&twist) {
                eprintln!("Publish error: {}", e);
            }
        }
        else if msg.is_binary() {
            eprintln!("Ignoring binary message.");
        }
        else if msg.is_close() {
            println!("Client closed the connection.");
            break;
        }
    }

    Ok(())
}

/// Convert [-1..1] track speeds to Twist. geometry_msgs::msg::Twist has f64 fields.
fn tracks_to_twist(left: f64, right: f64) -> Twist {
    let mut twist = Twist::default();
    let linear_scale = 0.5;
    let angular_scale = 1.0;

    // Simple differential-drive mapping
    twist.linear.x = ((left + right) / 2.0) * linear_scale;
    twist.angular.z = ((right - left) / 2.0) * angular_scale;

    twist
}
