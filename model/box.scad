$fn = 30;

// Dimensions of the base box shape
width = 95;
top_depth = 95;
depth = top_depth + 40;
height = 75;
front_height = height - 50;

fp_angle = 51.3; // todo: use atan or trigonometry to make this in sync

wall = 2.5;
delta = 0.01;

psocket_hole_span = 60;
psocket_hole_d    = 3;
psocket_key_w     = 2.3;
psocket_key_span  = 70;
psocket_hole_h    = 3;

// front panel
encoder_d = 7.2;

// display part dimensions
display_w = 34.0;
display_h = 18.2;

screw_hole_d = 3.2;

// display holes
disp_hole_d = screw_hole_d;

disp_hole_top = 3.0; // above display edge
disp_hole_bottom = 28.5; // relative to the top hole
disp_hole_span   = 30.5; // left to right, centered with display

// switch dimensions
switch_w = 19.5;
switch_ww = 20.1;
switch_h = 13;
switch_lip = 1.2;
switch_x = 15;
switch_depth = 22;

// power supply
ps_w = 31.8;
ps_d = 59;
ps_x = 64;
ps_y = 30;
ps_h = 2.5; // we have to rise the pcb of the ps this much in order to fit the bottom mounted parts

// solid state relay
ssr_w = 45;
ssr_d = 58;
ssr_h = 33;
ssr_x = 14;
ssr_y = 26;

// screw-in mounts
choc_w = 22;
choc_d = 20.5;
choc_h = 18;
choc_x = 99;
choc_y = 20;

// power in lead-in
powin_off = 15.5;
powin_w = 10;
powin_d = 29.5;
powin_cable_d = 7.5;
temp_cable_d = 3.9;
powin_c1_y = 8;
powin_c2_y = 16;
powin_c1_z = 1;
powin_c2_z = 0;

// pcb mount pillars
pcb_x = 101;
pcb_y = -44.5;
pcb_d = 8;
pcb_w = 10;
pcb_h = 22;

// real pcb dimensions
pcb_r_w = 49.3; // slightly more than pcb (48.8) so we can slide it in
pcb_r_l = 66.6;
pcb_r_h = 1.4;

/*

  TODO:
  * test fit:
    X display
    X encoder
    - cable leadin
    - probe leadout
    X switch cutout
    X screw fit

  * mount points
    X ssr
    - power supply
    X main board
    X chocolate

  * models
    - Beautiful knob
  */

module rounded_cube(dim = [], d = 4) {
    // TODO: Use union (4 cylinders, 2 cubes) instead. this is expensive without a good reason
    hull() {
        translate([d/2,d/2,0]) cylinder(d=d, h=dim[2]);
        translate([dim[0]-d/2,d/2,0]) cylinder(d=d, h=dim[2]);
        translate([d/2,dim[1]-d/2,0]) cylinder(d=d, h=dim[2]);
        translate([dim[0]-d/2,dim[1]-d/2,0]) cylinder(d=d, h=dim[2]);
    }
}

module power_socket() {
    translate([-40,-40,0]) difference() {
        union() {
            difference() {
                rounded_cube([80,80,9.8],d=10);
                translate([1.5,1.5,-1.5]) rounded_cube([77,77,9.8]);
            }
            translate([15/2,15/2,9.8]) rounded_cube([65,65,3.5]);
            translate([40,40,-3]) cylinder(d=42.3,h=16);
            translate([20,15,-25]) rounded_cube([40,50,25]);
        }

        translate([40,40,9.8+3.5-16+delta]) cylinder(d=38.5,h=16);
    }
}

// power socket frame - subtracted from the top panel to allow the socket
// to be slightly (1mm) inserted into the panel
module socket_frame(h=2) {
    wid = 81.5;
    iwid = 76;
    dif = wid-iwid;
    translate([-wid/2,-wid/2,0]) difference() {
        rounded_cube([wid,wid,h],d=10);
        translate([dif/2,dif/2,-0.1]) rounded_cube([iwid,iwid,h+0.2],d=9.8);
    }
}

module bracket(h=0.5) {
    br_w = 14.1;
    br_h = 22.5;

    difference() {
        union() {
            translate([br_w/2,0,h/2]) cube([br_w,br_h,h],center=true);
            translate([6.05,0,-wall-0.2]) cylinder(d=psocket_hole_d,h=wall,$fn=30);
        }
      //  translate([11.0+3.3/2,0,-delta+h/2]) cube([3.3,psocket_key_w,h+0.2],center=true);
    }
}

module mounting_brackets(h=0.5) {
    sp   = 47.9;

    // spaced
    for (a=[0,180]) rotate([0,0,a]) translate([sp/2,0,0]) bracket(h);
}

module socket_pillar() {
    extd = psocket_hole_d+2*wall;
    difference() {
        union() {
            cylinder(d=extd,h=psocket_hole_h);
            translate([0,0,psocket_hole_h-2]) cylinder(d2=extd+1,d1=extd,h=2);
        }
        translate([0,0,-0.1]) cylinder(d=psocket_hole_d,h=psocket_hole_h+0.2);
    }
}

module mounting_pillars(h=0.5) {
    sp   = psocket_hole_span;

    // spaced
    for (a=[0,180]) rotate([0,0,a]) translate([sp/2,0,0]) socket_pillar();
}

// positive geometry wall power socket mount, placed directly on the top surface
module psocket_mount() {
    rotate([0,0,90]) translate([0, -width/2, -wall]) union() {
        difference() {
            union() {
                cube([top_depth, width, wall]);
                // strengthening for the nuts/bolts
                translate([top_depth/2,width/2,-psocket_hole_h]) mounting_pillars();
            }
            translate([top_depth/2,width/2,0]) {
                translate([0,0,wall-1]) socket_frame();
                translate([-51/2,-51/2,-delta]) rounded_cube([51,51,10]);
                translate([0,0,wall-1]) mounting_brackets(h=1+delta);

            }
        }
    }
}

module side_shape() {
    polygon([
                    [0,0],
                    [0,height],
                    [top_depth, height],
                    [depth, front_height],
                    [depth, 0]
            ]);
}

module base_shape() {
    translate([0,width/2,0]) rotate([90,0,0]) linear_extrude(height=width) side_shape();
}


module inner_cavity() {
    translate([0,width/2,0]) difference() {
        w2 = 2*wall;
        union() {
            translate([0,-wall,0]) rotate([90,0,0]) linear_extrude(height=width-w2) offset(r=-wall) side_shape();
            translate([wall,-width+wall,-delta]) cube([depth-w2,width-w2,3*wall]);
        }

        for (x=[20,40,60,80]) {
            translate([x,0,height-20]) rotate([45,0,0]) cube([wall, 40, 40]);
            translate([x,-width-wall,height-20]) rotate([45,0,0]) cube([wall, 40, 40]);
        }
    }
}


//inner_cavity();

//translate([top_depth/2,0,height-1]) power_socket();


module display_cutter() {
    cube([display_w,display_h,2*wall]);
    for(y=[display_h+disp_hole_top,display_h-disp_hole_bottom+disp_hole_top]) {
        for(x=[0,disp_hole_span]) {
            translate([x+disp_hole_d/2,y,0]) cylinder(d=disp_hole_d,h=2*wall);
        }
    }
}


module encoder_cutter() {
    cylinder(d=encoder_d,h=2*wall);
}

module front_panel() {
/*    difference() {*/
//      cube([70,50,wall]);
    union() {
        translate([5,20,-delta]) display_cutter();
        translate([60,30,-delta]) encoder_cutter();
        translate([ 1, 48, wall - 0.15 ]) linear_extrude(height    = 1,
                                                         convexity = 4)
                text("Hrncátor 3100 Pro", font = "DejaVu Serif:style=Italic", size = 4);

        translate([5,17,wall-delta]) cube([display_w,1,1]);
  }
}

module raw_body() {
    align=3.2; // an ugly method for sure, but lets us do the positioning of front panel stuff easier
    union() {
        difference() {
            base_shape();
            inner_cavity();
            translate([0,-width/2,height-wall-delta]) cube([top_depth, width, wall+delta*2]);
//            translate([depth-align,-width/2+25/2,front_height]) rotate([0,0,90]) rotate([fp_angle,0,0]) translate([0,0,-delta]) cube([70,50,2*wall]);
            translate([depth-align,-width/2+25/2,front_height]) rotate([0,0,90]) rotate([fp_angle,0,0]) front_panel();
        }

        translate([top_depth/2,-width/2,height]) psocket_mount();
    }
}

// half of the cutters, mirrored later for accuracy
module cutter_half(offseting=0) {
    off = 5;
    translate([-1+depth/2,width/2+off/2,height + off/2]) rotate([45,0,0]) cube([depth,10-offseting,10-offseting],center=true);
    translate([off/2+top_depth,width/2+off/2,height - off/2]) rotate([0,52,0]) rotate([45,0,0]) cube([depth,10-offseting,10-offseting],center=true);


    translate([top_depth+2,width/2+off/2,height+3]) rotate([0,0,-90]) rotate([55,0,0]) cube([depth,10-offseting,10-offseting],center=true);


    translate([-1,width/2+off/2,height+3]) rotate([0,0,-90]) rotate([45,0,0]) cube([depth,10-offseting,10-offseting],center=true);

    translate([depth+5.3,width/2+off/2,front_height - off/2]) rotate([0,0,-90]) rotate([15,0,0]) cube([depth,10-offseting,10-offseting],center=true);

    // corners
    translate([0,width/2,0]) rotate([0,0,45]) translate([0,0,(height+1)/2]) cube([3.5-offseting,3.5-offseting,height+2],center=true);
    translate([depth,width/2,0]) rotate([0,0,45]) translate([0,0,(height+1)/2]) cube([3.5-offseting,3.5-offseting,height+2],center=true);
}

module cutters(off = 0) {
    cutter_half(off);
    mirror([0,1,0]) cutter_half(off);
}

module pillar(d,w,h,s=-1) {
    dw = d+2*w;
    union() {
        difference() {
            cylinder(d=dw,h=h);
            translate([0,0,-delta]) cylinder(d=d,h=h-dw+2*delta);
            if (s != 0) {
                translate([0,0,h]) rotate([0,-s*45,0]) cube([4*dw,dw,dw], center=true);
            }
        }
    }
}

module pillars(d=3,w=wall,h=22,en=1) {
    // all corners have screw pillars
    pd = 9 + wall;
    for (y = [-width/2+pd/2,width/2-pd/2]) {
        for (x = [pd/2,depth-pd/2]) {
            xs = x - depth/2;
            translate([x,y,0]) pillar(d=d,w=w,h=h,s=en*abs(xs)/xs);
        }
    }

}

module switch_hole() {
    translate([depth-wall-delta,-width/2+switch_x,-delta]) union() {
        cube([2*wall, switch_w, switch_h]);
        translate([-switch_depth+wall/2, -(switch_ww-switch_w)/2, 0]) cube([switch_depth, switch_ww, switch_h]);
    }
}

module cable_leadin(d,h1,h2) {
    union() {
        cylinder(d=d,h=h1);
        translate([0,0,h1-delta]) cylinder(d1=d,d2=0.1,h=h2);
    }
}

module power_input_clamp_holder() {
    translate([-delta,powin_off,-delta]) {
        difference() {
            translate([wall+0.25,-wall,wall]) cube([powin_w-0.75,2*wall,wall+delta*2]);
            hull() {
                translate([wall,0,0]) cube([powin_w,powin_d,wall-0.1]);
                translate([wall-delta,wall,wall]) cube([powin_w+delta*2,powin_d,wall+delta*2]);
            }
        }
    }
}

module power_input_clamp() {
    translate([-delta,powin_off,-delta]) difference() {
        union() {
            translate([wall,0,0]) cube([powin_w,powin_d-0.5,wall-0.3]);
            translate([wall,powin_c1_y,powin_c1_z]) rotate([0,90,0])  cable_leadin(d=powin_cable_d+2*wall,h1=powin_w,h2=0);
            translate([wall,powin_c2_y,powin_c2_z]) rotate([0,90,0]) cable_leadin(d=temp_cable_d+2*wall,h1=powin_w,h2=0);
        }

        translate([wall-delta,powin_c1_y,powin_c1_z]) rotate([0,90,0])  cable_leadin(d=powin_cable_d,h1=powin_w+2*delta,h2=0);
        translate([wall-delta,powin_c2_y,powin_c2_z]) rotate([0,90,0]) cable_leadin(d=temp_cable_d,h1=powin_w+2*delta,h2=0);
        translate([wall,0,-3*wall]) cube([powin_w,powin_d-0.5,3*wall]);

        // screw hole
        translate([0,-powin_off,0]) pillars(d=0,w=3/2,h=3*wall+2*delta,en=0);
    }
}

module power_input() {
    translate([-delta,powin_off,-delta]) {
        translate([wall,0,0]) cube([powin_w,powin_d,wall+delta*2]);
        translate([0,powin_c1_y,powin_c1_z]) rotate([0,90,0]) cable_leadin(d=powin_cable_d,h1=powin_w+wall,h2=3.2*wall);
        translate([0,powin_c2_y,powin_c2_z]) rotate([0,90,0]) cable_leadin(d=temp_cable_d,h1=powin_w+wall,h2=4*wall);

        // space for the cable under the relay, etc
        translate([13,3,-2.3]) cube([17,17,4]);
    }
}

module cable_holder_mounts() {
    hidex = 1;

    for (y=[-22,22]) {
        for (z=[25,55]) {
            translate([ hidex, y, z ]) rotate([ 0, 90, 0 ])
                cylinder(d = screw_hole_d, h = wall);
        }
    }
}

module top_body() {
    difference() {
        union() {
            difference() {
                raw_body();
                cutters();
                switch_hole();
            }

            intersection() {
                base_shape();
                difference() {
                    cutters(-wall*2);
                    cutters();
                }
            }

            pillars();
        }

        // this cuts down the height of the pillar, so that we can insert a clamp under it
        power_input();

        cable_holder_mounts();
    }
}

module ps_hole(d,h) {
    translate([1.5*ps_w/3,15/2,0]) cylinder(d=d, h=h);
}

module ps_holders() {
    w2 = 2*wall;
    translate([ 0, -ps_d - wall, 0 ]) difference() {
        cube([ ps_w, w2, ps_h + w2 ]);

        // center is free, for the cables etc
        translate([ps_w/3,-delta,0]) cube([ ps_w/3, w2+delta*2, ps_h + w2 + delta ]);

        // cutout for the board
        translate([-delta,wall,ps_h]) hull() {
            hh = 2.3;
            cube([ ps_w+delta*2, wall, wall ]);
            translate([0,hh,hh]) cube([ ps_w+delta*2, wall, wall ]);
        }
    }

    // the other side has a pillar with a cutout, and a screw hole that clamps the pcb securely
    difference() {
        translate([ps_w/3,-wall,0]) cube([ps_w/3,15,w2]);
        translate([0,1,delta]) power_source();
        translate([1.5*ps_w/3,15/2,0]) cylinder(d=screw_hole_d, h=15);
    }
}

module ps_clamp() {
    w2 = 2*wall;
    translate([0,0,0.2]) difference() {
        translate([ps_w/3-wall,-wall,wall]) cube([ps_w/3+w2,15,w2]);
        translate([ps_w/3-0.5,-wall,wall-delta]) cube([ps_w/3+1,16,wall+0.5]);
        translate([0,1,delta]) power_source();
        translate([0,1,-delta]) power_source();
        translate([1.5*ps_w/3,15/2,0]) cylinder(d=screw_hole_d, h=15);
    }
}

module power_source() {
    rotate([0,0,-90]) translate([0,0,ps_h]) cube([ps_d, ps_w, wall]);
}

module vent(l) {
    d = 1.5;
    hull() {
        translate([d/2,0,0]) cylinder(d=d,h=10);
        translate([l-d/2,0,0]) cylinder(d=d,h=10);
    }
}

module ssr_pillars(d=3,h=wall) {
    rotate([0,0,-90]) union() {
        translate([4.7,ssr_w/2,0]) cylinder(d=d,h=h);
        translate([4.7+49.6,ssr_w/2,0]) cylinder(d=d,h=h);
    }
}

module ssr() {
    rotate([0,0,-90]) difference() {
        cube([ssr_d, ssr_w, ssr_h]);
        translate([4.7,ssr_w/2,0]) cylinder(d=12,h=ssr_h+delta);
        translate([4.7+49.6,ssr_w/2,0]) cylinder(d=12,h=ssr_h+delta);
    }
}

module choc_pillars(d,h) {
    rotate([0,0,-90]) union() {
        translate([choc_d/2,choc_w/3+choc_w/6,0]) cylinder(d=d,h=h);
    }
}

module choc() {
    rotate([0,0,-90]) union() {
        cube([choc_d, choc_w/3, choc_h]);
        translate([choc_d/2-choc_w/6,choc_w/3,0]) cube([choc_w/3, choc_w/3, choc_w/3]);
        translate([0,choc_w*2/3,0]) cube([choc_d, choc_w/3, choc_h]);
    }
}

module pcb_pillars() {
    difference() {
        union() {
            cube([pcb_w,pcb_d,pcb_h]);
            translate([-pcb_r_w+pcb_w-wall,66.6+20-pcb_d+wall,0]) cube([pcb_w,pcb_d,pcb_h]);
        }
        translate([pcb_w/2,pcb_d/2,0]) cylinder(d=screw_hole_d-0.2,h=pcb_h+delta);
        translate([-pcb_r_w+pcb_w-wall,66.6+20-pcb_d+wall,0])
            translate([pcb_w/2,pcb_d/2,0]) cylinder(d=screw_hole_d-0.2,h=pcb_h+delta);
    }
}

module pcb() {
    p2 = pcb_r_h*2;

    hull() {
        cube([pcb_r_w,pcb_r_l,pcb_r_h]);
        translate([pcb_r_h,pcb_r_h,pcb_r_h]) cube([pcb_r_w-p2,pcb_r_l-p2,pcb_r_h]);
    }
}

module pcb_frame() {
    fr_w = pcb_r_w+wall;
    fr_l = pcb_r_l+pcb_w+wall;
    fr_h = 1.4+wall+2;

    translate([-pcb_r_w+pcb_w,0,0]) {
        w2 = 2*wall;
        difference() {
            cube([fr_w,fr_l+10,fr_h]);

            // screw holes
            translate([fr_w-pcb_w/2,pcb_d/2,-delta]) cylinder(d=screw_hole_d,h=4*wall);
            translate([pcb_w/2,fr_l + 10 - pcb_d/2,-delta]) cylinder(d=screw_hole_d,h=4*wall);

            translate([-delta,-delta,wall]) cube([fr_w+2*delta,8,fr_h+delta]);
            translate([-delta,-delta,-delta/2]) cube([fr_w-pcb_w+2*delta,8,fr_h+delta]);

            translate([-delta,fr_l+delta,wall]) cube([fr_w+2*delta,10,fr_h+delta]);

            // cutout so we can slide the pcb in
            translate([wall-delta,fr_l-wall-delta,wall]) cube([fr_w-w2+2*delta,10,fr_h+delta]);
            translate([pcb_w+delta,fr_l+delta,-delta/2]) cube([fr_w-pcb_w+2*delta,10,fr_h+delta]);

            translate([wall/2,10,wall]) pcb();
            // second instance to have a slide-in possible
            translate([wall/2,15,wall]) pcb();

            // main cutout. We leave some material for rigidity
            translate([wall,10,-delta]) difference() {
                cube([fr_w-w2,fr_l-10-wall,fr_h+w2]);
                /*
                translate([fr_w-5,-2,0]) rotate([0,0,35]) cube([wall,fr_l+5,wall/2]);
                translate([-3,-2,0]) rotate([0,0,-35]) cube([wall,fr_l+10,wall/2]);
                */
            }
        }

        translate([wall/2,10,wall]) %pcb();
    }
}


module bottom_piece() {
    w2=2*wall;
    sp = 0.25;

    //

    union() {
        difference() {
            union() {
                translate([0,0,-wall*1.5])
                    difference() {
                    translate([0,-width/2,0]) cube([depth,width,wall*1.5]);
                    cutters();
                    translate([0,0,wall*1.5]) power_input();
                }

                difference() {
                    translate([wall+sp,wall+sp-width/2,0]) cube([depth-w2-2*sp,width-w2-2*sp,wall]);
                    /*            echo("build space");
                                  echo(depth-2*wd);
                                  echo(width-2*wd);
                    */
                    translate([wd,-width/2+wd,-delta]) cube([depth - 2*wd, width - 2*wd, wall+delta*2]);
                    translate([0,0,-delta]) pillars(d=0,w=1.75+wall+sp);

                    // power input mount
                    switch_hole();
                    power_input();
                }

                // pillars for some components
                translate([ssr_x,ssr_y,0]) ssr_pillars(10,5);
                translate([choc_x,choc_y,0]) choc_pillars(10,2);
                translate([ps_x,ps_y,0]) ps_hole(10,2);
            }

            translate([0,0,-wall*1.5-delta]) {
                translate([0,0,2.4+0.2]) pillars(d=0,w=3/2,h=3*wall+2*delta,en=0);
                pillars(d=0,w=6/2,h=2.4,en=0);
                // holes in the ssr pillars for screws
                translate([0,0,2.4+0.2]) union() {
                    hh = 3*wall+2*delta;
                    translate([ssr_x,ssr_y,0]) ssr_pillars(3,h=hh);
                    translate([choc_x,choc_y,0]) choc_pillars(3,h=hh);
                    translate([ps_x,ps_y,0]) ps_hole(3,h=hh);
                }

                translate([ssr_x,ssr_y,0]) ssr_pillars(6,h=2.4);
                translate([choc_x,choc_y,0]) choc_pillars(6,h=2.4);
                translate([ps_x,ps_y,0]) ps_hole(6,h=2.4);
            }

            wd = 12;

            // vent holes for the SSR
            for (y = [12:3:ssr_d-12]) {
                translate([ssr_x,ssr_y-y,-5]) vent(l=ssr_w);
            }

            for (y = [12:3:ps_d-12]) {
                translate([ps_x,ps_y-y,-5]) vent(l=ps_w);
            }
        }

        // pcb mount pillar
        translate([pcb_x+wall,pcb_y,0]) pcb_pillars();

        // power supply holders
        translate([ps_x,ps_y,0]) ps_holders();

        power_input_clamp_holder();
        %power_input_clamp();

        %translate([ps_x,ps_y,0]) ps_clamp();

        // display where the switch is, for clearances
        %switch_hole();

        // components on the bottom - checking clearances
        %translate([ssr_x, ssr_y, 5]) ssr();
        %translate([ps_x, ps_y, 0]) power_source();
        %translate([choc_x, choc_y, 0]) choc();
        %translate([pcb_x,pcb_y,pcb_h]) pcb_frame();
    }
}


/*
difference() {
    translate([0,7,0]) cube([60,38,wall]);
    front_panel();
}
*/

%translate([width/2,0, height]) power_socket();

top_body();

// DONE:
// pcb_frame();
// rotate([0,180,0]) ps_clamp();
// power_input_clamp();
// bottom_piece();
