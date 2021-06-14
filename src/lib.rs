extern crate nalgebra as na;

pub mod generators;
pub mod io;
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
