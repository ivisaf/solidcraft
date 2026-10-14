//! Ordered parallel map over independent items, on the process-wide worker pool (desktop
//! only; the browser build runs serially). Each item runs under [`crate::guard`], so a panic in
//! one item is that item's error and never reaches the pool.

use crate::{Result, guard};

/// `f` over `items`, results in item order: on the pool when `parallel`, else on the calling
/// thread. Waking the pool's workers costs tens of microseconds, so callers ask for it only
/// when the items carry more work than that (decided from the work, not the item count).
pub(crate) fn map<T: Sync, R: Send>(what: &'static str, items: &[T], parallel: bool, f: impl Fn(&T) -> Result<R> + Sync) -> Vec<Result<R>> {
    let one = |item: &T| guard(what, || f(item));
    // The browser build has no worker pool: everything runs on the calling thread.
    #[cfg(target_arch = "wasm32")]
    let _ = parallel;
    #[cfg(not(target_arch = "wasm32"))]
    {
        use rayon::prelude::*;
        if parallel && items.len() > 1 && rayon::current_num_threads() > 1 {
            return items.par_iter().map(one).collect();
        }
    }
    items.iter().map(one).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::KernelError;

    #[test]
    fn keeps_order_and_errors() {
        let items: Vec<u32> = (0..1000).collect();
        let out = map("test", &items, true, |x| if *x == 500 { Err(KernelError::Invalid("five hundred".into())) } else { Ok(x * 2) });
        assert_eq!(out.len(), 1000);
        for (i, r) in out.iter().enumerate() {
            match r {
                Ok(v) => assert_eq!(*v, 2 * i as u32),
                Err(_) => assert_eq!(i, 500),
            }
        }
    }
}
