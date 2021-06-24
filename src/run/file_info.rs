use std::path::PathBuf;

pub struct FileInfo {
    path: PathBuf,
}

impl FileInfo {
    pub fn new(path: PathBuf) -> Self {
        Self {
            path,
        }
    }

    pub fn frame_number(&self) -> usize {
        let path = self.path_as_string();
        let frame = path
            .split('.')
            .collect::<Vec<&str>>()
            .first()
            .unwrap()
            .to_string();
        let frame = frame
            .split('-')
            .collect::<Vec<&str>>();
            
        let frame = frame.last().unwrap();

        let frame = frame.parse::<usize>().unwrap();
        
        frame
    }

    fn path_as_string(&self) -> String {
        self.path.to_string_lossy().to_string()
    }
}

#[cfg(test)]

mod tests {
    use super::*;

    #[test]
    fn test_file_frame_works() {
        let file_name = "particles-100.ply";
        let file_info = FileInfo::new(PathBuf::from(file_name));
        assert_eq!(100, file_info.frame_number())

    }
}

