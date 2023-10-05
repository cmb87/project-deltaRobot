
difference(){
   union(){
   translate([0,0,0])cylinder(r=29.95/2, h=15, $fn=40);
   translate([0,0,13.5])cylinder(r=33/2, h=15-13.5+3, $fn=40);
       
   }
 translate([-20.05/2,-20.05/2,-1])cube([20.05,20.05,42]);   
}