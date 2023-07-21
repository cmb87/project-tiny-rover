
difference(){
    
  hull(){
  translate([-71.1/2,-41.7/2,0])cube([71.1,41.7,2]);
  translate([-60/2,-41.7/2,2+8.1-1])cube([60,41.7,1]); 
  }
    
  translate([-22/2-5.25,-6/2-16,-1])cube([22,6,50]);
  translate([71.1/2-2.5,0,-1])cylinder(r=1.3,h=50,$fn=40);
  translate([-71.1/2+2.5,0,-1])cylinder(r=1.3,h=50,$fn=40);
  
  translate([-11/2+10,-6/2+16,-1])cube([11,6,50]);
translate([10+7.5,6+10,-1])cylinder(r=1,h=50,$fn=40);
translate([10-7.5,6+10,-1])cylinder(r=1,h=50,$fn=40);
  
  translate([-90/2,+17,5])rotate([0,90,0])cylinder(r=2.5,h=90,$fn=30);
  translate([-90/2,-17,5])rotate([0,90,0])cylinder(r=2.5,h=90,$fn=30);
  translate([71.1/2-2.5,0,3])cylinder(r=3.1,h=50,$fn=40);
  translate([-71.1/2+2.5,0,3])cylinder(r=3.1,h=50,$fn=40);
  
  translate([71.1/2-10.5,-17,-1])cylinder(r=1.2,h=50,$fn=40);
  translate([71.1/2-10.5,+17,-1])cylinder(r=1.2,h=50,$fn=40);

  translate([71.1/2-58.5,-17,-1])cylinder(r=1.2,h=50,$fn=40);
  translate([71.1/2-58.5,+17,-1])cylinder(r=1.2,h=50,$fn=40); 
  translate([-11/2-5,-6/2+16,-1])cube([5,6,50]);
  
  color("red")translate([-64/2-4,-26.1/2,2])cube([64,26.1,8.2]);
  
}

