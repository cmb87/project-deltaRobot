
if (true){ 
difference(){
    

  hull(){
    translate([-3,-28/2,-10/2])cube([3,28,10]);
    translate([18-2*3,-28/2-7,-10/2])cube([3,28,10]);
  }
    
    
  translate([-0.1,-5,0])rotate([0,90,0])cylinder(r=3/2, h=7, $fn=40);
  translate([-0.1,+5,0])rotate([0,90,0])cylinder(r=3/2, h=7, $fn=40);
  
  translate([-20,-20/2,-40/2])cube([20,20,40]);
  translate([-20/2+20,-20/2-7,-40/2])cube([20,20,40]);
  translate([10.1-7,-5-7,0])rotate([0,90,0])cylinder(r=3/2, h=7, $fn=40);
  translate([10.1-7,+5-7,0])rotate([0,90,0])cylinder(r=3/2, h=7, $fn=40);
}

}

