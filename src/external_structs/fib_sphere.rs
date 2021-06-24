
use std::time::Duration;

use serde::{Serialize, Deserialize};

#[derive(Clone, Serialize, Deserialize, Default)]
pub struct TreeState {
    pub npoints: u32,
    pub ratio: f32,
    pub radius: f32,
}

#[derive(Serialize, Deserialize)]
pub struct StateHistory {
    pub history: Vec<StateInstance>,
}

impl StateHistory {
    pub fn new() -> Self {
        Self {
            history: Vec::new(),
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct StateInstance {
    pub time_offset: std::time::Duration,
    pub state: TreeState,
}

impl StateInstance {
    pub fn new(time_offset: Duration, state: TreeState) -> Self {
        Self {
            time_offset,
            state
        }
    }
}