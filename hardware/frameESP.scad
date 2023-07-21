
difference(){
  union() {
  hull(){
    translate([-59/2,-20.5/2,0])cube([59,20.5,0.1]);
    translate([-40/2+5,-20.5/2,10])cube([40,20.5,0.1]);
  }
   translate([-8/2+71.1/2-10.5,-35/2,0])cube([8,35,3]);
   translate([-8/2+71.1/2-58.5,-35/2,0])cube([8,35,3]);
   translate([71.1/2-10.5,-17,0])cylinder(r=4,h=3,$fn=40); 
   translate([71.1/2-10.5,+17,0])cylinder(r=4,h=3,$fn=40);
   translate([71.1/2-58.5,-17,0])cylinder(r=4,h=3,$fn=40);
   translate([71.1/2-58.5,+17,0])cylinder(r=4,h=3,$fn=40); 
   //translate([22,0,5])rotate([0,90,0])cylinder(r=5,h=7,$fn=30);
  }
 
  translate([-38/2+3,-18/2,-0.1])cube([38,18,8]);
  translate([71.1/2-10.5,-17,-1])cylinder(r=1.2,h=50,$fn=40);
  translate([71.1/2-10.5,+17,-1])cylinder(r=1.2,h=50,$fn=40);

  translate([71.1/2-58.5,-17,-1])cylinder(r=1.2,h=50,$fn=40);
  translate([71.1/2-58.5,+17,-1])cylinder(r=1.2,h=50,$fn=40); 
  
  translate([-11/2+ 0,50/2,2.8])rotate([90,0,0])cylinder(r=2.5,h=50,$fn=40);
  translate([-11/2+10 ,50/2,2.8])rotate([90,0,0])cylinder(r=2.5,h=50,$fn=40);
  translate([-11/2+20,50/2,2.8])rotate([90,0,0])cylinder(r=2.5,h=50,$fn=40);
  
    translate([20,5,-1])cylinder(r=1,h=50,$fn=40);
    translate([20,-5,-1])cylinder(r=1,h=50,$fn=40);

    translate([10,5,-1])cylinder(r=1,h=50,$fn=40);
    translate([10,-5,-1])cylinder(r=1,h=50,$fn=40);

    translate([0,5,-1])cylinder(r=1,h=50,$fn=40);
    translate([0,-5,-1])cylinder(r=1,h=50,$fn=40);

    translate([-10,5,-1])cylinder(r=1,h=50,$fn=40);
    translate([-10,-5,-1])cylinder(r=1,h=50,$fn=40);
    
//translate([20,5,5.5])rotate([0,90,0])cylinder(r=1,h=30,$fn=40);
//translate([20,-5,5.5])rotate([0,90,0])cylinder(r=1,h=30,$fn=40);

translate([-10,-40/2,-0.1])cube([30,40,2.5]);
translate([20,0,4.5])rotate([0,90,0])cylinder(r=3.5,h=10,$fn=30);
translate([25,-11/2,10])rotate([0,90,0])cube([11,11,12]);
}





