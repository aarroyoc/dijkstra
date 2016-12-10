extern crate nalgebra as na;

use na::*;

#[test]
fn dijkstra_table_test() {
	let i = i32::max_value();
	let matrix = DMatrix::from_row_vector(7,7,&[
		i,3,9,i,i,i,i,
		3,i,2,7,1,i,i,
		9,2,i,7,1,i,i,
		i,7,7,i,5,2,8,
		i,1,1,5,i,9,i,
		i,i,i,2,9,i,4,
		i,i,i,8,i,4,i
	]);
	let final_matrix = DMatrix::from_row_vector(6,7,&[
		i,3,9,i,i,i,i,
		i,3,5,10,4,i,i,
		i,3,5,9,4,13,i,
		i,3,5,9,4,13,i,
		i,3,5,9,4,11,17,
		i,3,5,9,4,11,15
	]);
	let dijkstra_matrix=dijkstra_table_gen(&matrix,0).0;
	assert!(dijkstra_matrix == final_matrix,"Dijkstra Matrix not equal to test matrix");
}

#[test]
fn dijkstra_path_test() {
	let i = i32::max_value();
	let matrix = DMatrix::from_row_vector(7,7,&[
		i,3,9,i,i,i,i,
		3,i,2,7,1,i,i,
		9,2,i,7,1,i,i,
		i,7,7,i,5,2,8,
		i,1,1,5,i,9,i,
		i,i,i,2,9,i,4,
		i,i,i,8,i,4,i
	]);
	let vector = DVector::from_slice(6,&[0,1,4,3,5,6]);
	let path = dijkstra_path(&matrix,0,6);
	println!("{:?}",path.0);
	assert!(path.0 == vector,"Dijkstra path not equal to test path");
	println!("Min is: {}",path.1);
	assert!(path.1 == 15, "Minimum weight is wrong");
}

fn print_matrix(matrix: &DMatrix<i32>) -> () {
	println!("");
	let i: i32 = i32::max_value();
	for row in 0..matrix.nrows() {
		for item in matrix.row(row).iter() {
			if *item == i {
				print!("{}","-");
			}else{
				print!("{}",item);
			};
		}
		println!("");
	}
}

fn min_of_array_and_not_in_array(row: &DVector<i32>, path: &Vec<usize>) -> i32 {
	let mut min = i32::max_value();
	for i in 0..row.len() {
		match path.iter().position(|&r| r == (i as usize)) {
			None => {
				min = na::min(min,row[i]);
			},
			Some(_) => {}
		}
	}
	min
}

fn index_of_int_array(array: &DVector<i32>, key: i32) -> i32 {
	let mut returnvalue = -1;
	for i in 0..array.len() {
	    if key == array[i] {
	        returnvalue = i as i32;
	        break;
	    }
	}
	returnvalue
}

fn index_of_int_vector(array: &[usize], key: usize) -> i32{
	let mut returnvalue = -1;
	for i in 0..array.len() {
	    if key == array[i] {
	        returnvalue = i as i32;
	        break;
	    }
	}
	returnvalue
}

/// Generates a table for Dijkstra algorithm
/// # Arguments
/// * `pesos` - A weight matrix (must be a square matrix)
/// * `start` - The starting node
/// # Returns
/// A tuple with two values
/// 0 - The Dijkstra matrix
/// 1 - A list containing the order of nodes used to make the minimum paths

pub fn dijkstra_table_gen(pesos: &DMatrix<i32>, start: usize) -> (DMatrix<i32>,DVector<usize>) {
	let mut matrix: DMatrix<i32> = DMatrix::new_zeros(pesos.nrows() -1, pesos.nrows());
	let mut path: Vec<usize> = Vec::new();
	path.push(start);
	let mut row = pesos.row(start);
	
	for i in 0..pesos.nrows() -1 {
		matrix.set_row(i,row.clone());
		let cost = min_of_array_and_not_in_array(&row,&path);
		if index_of_int_array(&row,cost) > -1 {
			let pos: usize = index_of_int_array(&row,cost) as usize;
			path.push(pos);
		
			for j in 0..row.len() {
				if index_of_int_vector(&path,j) < 0 {
					if pesos[(pos,j)] != i32::max_value() {
						row[j] = na::min(row[j],cost + pesos[(pos,j)]);
					}
				}
			}
		}

	}
	(matrix, DVector::from_slice(path.len(),&path))
}

/// Calculates the minimum path using Dijkstra algorithm
/// # Arguments
/// * `matrix` : a weight matrix (must be a square matrix)
/// * `start` : the starting node for the path
/// * `end` : the end node for the path
/// # Returns
/// Tuple with two values
/// * 0 - a DVector containing a list of nodes used to do the path, from start to the end
/// * 1 - an i32 with the minimum weight

pub fn dijkstra_path(matrix: &DMatrix<i32>, start: usize, end: usize) -> (DVector<usize>,i32) {
	let dijkstra = dijkstra_table_gen(&matrix,start);
	let mut path: Vec<usize> = Vec::new();
	path.insert(0,end);
	
	let mut column = dijkstra.0.column(end);
	
	let mut c = column.len() - 1;
	let mut p = dijkstra.1.len() - 2;
	
	let min_weight = column[c];
	
	while c > 0{
		if column[c] == column[c-1] {
			c -= 1;
			p -= 1;
		}else {
			let point = dijkstra.1[p];
			path.insert(0,point);
			column = dijkstra.0.column(point);
		}
	}
	
	path.insert(0,start);
	(DVector::from_slice(path.len(),&path),min_weight)
}