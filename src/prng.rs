// Simple Xorshift32 to keep zero external dependencies.
pub(crate) struct Prng {
    state: u32,
}

impl Prng {
    /// Constructs a PRNG with a custom non-zero seed.
    pub(crate) fn new(seed: u32) -> Self {
        Self {
            // Guard against 0 seed
            state: if seed == 0 { 0x9E37_79B9 } else { seed },
        }
    }

    pub(crate) fn next_u32(&mut self) -> u32 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.state = x;
        x
    }

    /// Fisher-Yates shuffle.
    pub(crate) fn shuffle<T>(&mut self, slice: &mut [T]) {
        for i in (1..slice.len()).rev() {
            let j = (self.next_u32() as usize) % (i + 1);
            slice.swap(i, j);
        }
    }
}
