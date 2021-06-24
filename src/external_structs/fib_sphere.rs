
use serde::{Serialize, Deserialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct TreeState {
    pub npoints: u32,
    pub ratio: f32,
    pub radius: f32,
}

#[derive(Serialize, Deserialize)]
pub struct StateHistory(Vec<StateInstance>);

impl StateHistory {
    pub fn new() -> Self {
        Self {
            0: Vec::new(),
        }
    }

    pub fn push(&mut self, instance: StateInstance) {
        self.0.push(instance)
    }

    pub fn get(&self,  idx: usize) -> Option<&StateInstance>{
        self.0.get(idx)
    }
}
#[derive(Serialize, Deserialize)]
pub struct StateInstance {
    pub time_offset: std::time::Duration,
    pub state: TreeState,
}

impl StateInstance {
    // pub fn new(from: &Time, state: TreeState) -> Self {
    //     let time_offset = std::time::Instant::now() - from.startup();
    //     Self {
    //         time_offset,
    //         state
    //     }
    // }
}