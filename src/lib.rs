extern crate nalgebra as na;

pub mod generators;
pub mod io;
pub mod run;
pub mod external_structs;
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
