from geometry import * 
from threejs_group import *

if __name__ == "__main__":

	viz_out = threejs_group()
	viz_out.js_dir = "../js/"

	viz_out.add_axis([1,2,3], [1,0,0,0], 1)
	viz_out.add_axis([1,5,2], [0.707,0,0.707,0],  1)

	viz_out.to_html("out/axis.html");
