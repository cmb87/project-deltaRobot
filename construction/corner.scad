

difference(){
hull(){
translate([0,0,0])translate([0,-30/2,0])cube([40,30,30]);      
rotate([0,0,60])translate([0,-30/2,0])cube([40,30,30]); 
rotate([0,0,30])translate([-30/2,-30/2,0])cube([30,30,40]); 
}

translate([0,0,(30-20.2)/2])translate([0,-20.2/2,0])cube([50,20.2,20.2]);  
translate([0,0,(30-20.2)/2])rotate([0,0,60])translate([0,-20.2/2,0])cube([50,20.2,20.2]);  
rotate([0,0,30])translate([-20.2/2,-20.2/2,2.5])cube([20.2,20.2,50]); 

rotate([0,0,60])translate([30,0,-1])cylinder(r=1.2,h=50, $fn=40); 
rotate([0,0,0])translate([30,0,-1])cylinder(r=1.2,h=50, $fn=40); 

rotate([0,0,60])translate([30,0,30])cylinder(r=2.5,h=50, $fn=40); 
rotate([0,0,0])translate([30,0,30])cylinder(r=2.5,h=50, $fn=40); 

}

