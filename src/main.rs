use reach_me_if_you_can::app::App;
use reach_me_if_you_can::config;

fn window_conf() -> macroquad::prelude::Conf {
    config::window_conf()
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut app = App::new();
    app.run().await;
}