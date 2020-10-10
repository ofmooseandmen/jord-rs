use std::fs::File;
use std::io::{self, BufRead, Error, ErrorKind};
use std::path::Path;

#[derive(Debug)]
pub struct Text(Vec<String>);

impl Text {
    pub fn from_file_content<P>(filename: P) -> io::Result<Text>
    where
        P: AsRef<Path>,
    {
        let file = File::open(filename)?;
        let content = io::BufReader::new(file).lines();
        let lines = content.filter_map(Result::ok).collect();
        Ok(Text(lines))
    }

    pub fn is_eot(&self) -> bool {
        self.0.is_empty()
    }

    pub fn skip_empty(&self) -> Self {
        Text(
            self.0
                .iter()
                .skip_while(|l| l.is_empty())
                .map(|s| s.to_string())
                .collect(),
        )
    }

    pub fn next(&self) -> io::Result<(String, Text)> {
        match self.0.first() {
            None => Err(Error::new(ErrorKind::UnexpectedEof, "expected more")),
            Some(s) => Ok((s.to_string(), Text(self.0[1..].to_vec()))),
        }
    }

    pub fn next_if_prefixed(&self, prefix: &str) -> io::Result<(String, Text)> {
        match self.0.first().and_then(|s| s.strip_prefix(prefix)) {
            None => Err(Error::new(
                ErrorKind::UnexpectedEof,
                format!("expected {}, found {:?}", prefix, self.0.first()),
            )),
            Some(s) => Ok((s.to_string(), Text(self.0[1..].to_vec()))),
        }
    }
}
