//  union(){
//    translate([-42/2,-42/2-15,-0-2])cube([42,15,15]);
//    translate([-42/2,+42/2,-0-2])cube([42,15,15]);
//    translate([-42/2,-42/2,-0-2])cube([42,42,4]);
//  }
//  
//          translate([-50/2,-42/2-10-2,0.5])cube([50,10.1,10.1]);
//        translate([-50/2,+42/2+2.5 ,0.5])cube([50,10.1,10.1]);
//  
  
  
 difference(){
hull(){
translate([0,0,0])translate([0,-16/2,0])cube([20,16,16]);      
rotate([0,0,60])translate([0,-16/2,0])cube([20,16,16]); 

}

translate([0,0,(16-10.1)/2])translate([0,-10.1/2,0])cube([50,10.1,10.1]);  
translate([0,0,(16-10.1)/2])rotate([0,0,60])translate([0,-10.1/2,0])cube([50,10.1,10.1]); 




}
