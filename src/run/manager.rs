use human_hash::humanize;
use std::path::PathBuf;
use uuid::Uuid;
use std::fs;
use glob::glob;

const PARTICLES_PATH: &str = "particles";
const PARTICLES_FULL_PATH: &str = "full";
const PARTICLES_SAMPLED_PATH: &str = "sampled";
const PARTICLES_SURFACE_PATH: &str = "surface";
const PARTICLES_PRUNED_PATH: &str = "pruned";

const METADATA_PATH: &str = "metadata";
const WAVE_DETECTION_PATH: &str = "wave-detection";

const CONFIG_FILE: &str = "config.toml";
const LOG_FILE: &str = "salvatore.log";

pub struct Manager {
    pub name: String,
    pub root: PathBuf,
}

impl Manager {
    pub fn new(name: Option<String>, root: PathBuf) -> Self {

        let name = match name {
            Some(name) => name,
            None => humanize(&Uuid::new_v4(), 4),
        };

        Self {
            name,
            root,
        }
    }

    pub fn from_run_path(run_dir: PathBuf) -> Self {
        let name = run_dir.file_name().unwrap().to_string_lossy().to_string();
        let root = run_dir.parent().unwrap();
        Self::new(Some(name), root.into())
    }

    pub fn from_config_file(config_file: PathBuf) -> Self {
        //FIXME: test that file exists
        Self::from_run_path(config_file.parent().unwrap().into())
    }

    pub fn run_path(&self) -> PathBuf {
        self.root.join(self.name.clone())
    }

    pub fn particles_path(&self) -> PathBuf {
        self.run_path().join(PARTICLES_PATH)
    }

    pub fn particles_full_path(&self) -> PathBuf {
        self.particles_path().join(PARTICLES_FULL_PATH)
    }

    pub fn particles_sampled_path(&self) -> PathBuf {
        self.particles_path().join(PARTICLES_SAMPLED_PATH)
    }

    pub fn particles_pruned_path(&self) -> PathBuf {
        self.particles_path().join(PARTICLES_PRUNED_PATH)
    }

    pub fn particles_surface_path(&self) -> PathBuf {
        self.particles_path().join(PARTICLES_SURFACE_PATH)
    }

    pub fn metadata_path(&self) -> PathBuf {
        self.run_path().join(METADATA_PATH)
    }

    pub fn wave_detection_path(&self) -> PathBuf {
        self.metadata_path().join(WAVE_DETECTION_PATH)
    }

    pub fn config_path(&self) -> PathBuf {
        self.run_path().join(CONFIG_FILE)
    }

    pub fn log_path(&self) -> PathBuf {
        self.run_path().join(LOG_FILE)
    }

    pub fn glob_path_sorted_string <'a> (&self, the_glob: &'a PathBuf) ->  Vec<String> {
        let mut entries: Vec<String> = glob(&the_glob.to_string_lossy())
        .expect("Particles path glob failed")
        .map(|entry| entry.unwrap().file_name().unwrap().to_str().unwrap().to_owned())
        .collect();
    
        alphanumeric_sort::sort_str_slice(entries.as_mut());

        entries
    }
    pub fn glob_path_sorted <'a> (&self, the_glob: &'a PathBuf) ->  Vec<PathBuf> {
        let entries = self.glob_path_sorted_string(the_glob);

        let parent = the_glob.parent().unwrap();
        entries
            .iter()
            .map(|name| parent.join(name))
            .collect()
    }

    pub fn create_run_path(&self) {
        self.create_path(self.run_path());
    }


    pub fn create_particles_path(&self) {
        self.create_path(self.particles_path());
    }

    pub fn create_path(&self, path: PathBuf) {
        match fs::create_dir_all(path.clone()) {
            Ok(_) => (),
            Err(_) => panic!("Could not create the path {}", path.to_string_lossy()),
        };
    }

}