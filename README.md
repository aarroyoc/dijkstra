# Dijkstra crate for Rust

Generate minimum paths using Dijkstra algorithm.

## Example

```
extern crate nalgebra as na;
extern crate dijkstra;

use na::*;
use dijkstra::*;

fn main(){
	let i = i32::max_value(); // i means infinity or no connection between nodes
	let matrix = DMatrix::from_row_vector(7,7,&[
		i,3,9,i,i,i,i,
		3,i,2,7,1,i,i,
		9,2,i,7,1,i,i,
		i,7,7,i,5,2,8,
		i,1,1,5,i,9,i,
		i,i,i,2,9,i,4,
		i,i,i,8,i,4,i
	]);
	let path = dijkstra_path(&matrix,0,6); // Going to Node 0 to node 6
	println!("{:?}",path.0); // [0,1,4,3,5,6] nodes used and ordered to build the minimum path
	println!("{:?}",path.1); // 15 (minimum weight)
}

```