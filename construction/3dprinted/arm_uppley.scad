
difference(){
 
 union(){
 hull(){
     translate([0,0,0])cylinder(r=60/2, h=30, $fn=40);
     translate([-30/2+40,-30/2,0])cube([30,30,30]);      
 }
 translate([-30,0,0])cylinder(r=3, h=30, $fn=40);
 }
 translate([0,0,-1])cylinder(r=47.1/2, h=32, $fn=40);
 translate([30,-20/2,(30-20)/2])cube([40,20,20]);  
 
translate([0,-53/2,-2])cylinder(r=2.2/2, h=60, $fn=40);
translate([0,+53/2,-2])cylinder(r=2.2/2, h=60, $fn=40);
translate([-53/2,0,-2])cylinder(r=2.2/2, h=60, $fn=40);
translate([+53/2,0,-2])cylinder(r=2.2/2, h=60, $fn=40);
 
translate([50-15,7,-2])cylinder(r=2.2/2, h=60, $fn=40);
translate([50-15,-7,-2])cylinder(r=2.2/2, h=60, $fn=40);
translate([50,7,-2])cylinder(r=2.2/2, h=60, $fn=40);
translate([50,-7,-2])cylinder(r=2.2/2, h=60, $fn=40);
 
translate([50-15,+7,27])cylinder(r=5/2, h=10, $fn=40); 
translate([50-15,-7,27])cylinder(r=5/2, h=10, $fn=40); 
translate([50,+7,27])cylinder(r=5/2, h=10, $fn=40); 
translate([50,-7,27])cylinder(r=5/2, h=10, $fn=40);  
 
translate([50-15,+7,-10+3])cylinder(r=5/2, h=10, $fn=40); 
translate([50-15,-7,-10+3])cylinder(r=5/2, h=10, $fn=40); 
translate([50,+7,-10+3])cylinder(r=5/2, h=10, $fn=40); 
translate([50,-7,-10+3])cylinder(r=5/2, h=10, $fn=40); 
}



difference(){
translate([0,0,11])cylinder(r=60/2, h=30-2*11, $fn=40);
translate([0,0,-1])cylinder(r=44/2, h=32, $fn=40);
}

