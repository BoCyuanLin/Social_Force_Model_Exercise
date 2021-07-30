// Project: Implementation for social force model
#include "libs.h"
#include "socialforce.h"
#include "gldrawSFM.h"

void ped2_crossing();
void ped3_crossing();
void ped_wall_crossing();
void peds_wall_crossing();
void peds_gate_crossing();
void peds_aisle_crossing_60();

int main(int argc, char* argv[]){

    glutInit(&argc, argv);

    //ped2_crossing();

    //ped3_crossing();

    //ped_wall_crossing();

    //peds_wall_crossing();

    //peds_gate_crossing();

    peds_aisle_crossing_60();

    system("pause");
}


void ped2_crossing() {
    int step_num = 50;
    std::vector<std::vector<double>> initial_states{
        { -5.0, -5.0, 0.5, 0.5, 5.0, 5.0},
        { 5.0, -4.7, -0.5, 0.5, -5.0, 5.0}
    };
    std::vector<std::vector<double>> temp_states;
    std::vector<std::vector<std::vector<double>>> states_steps;

    socialforcemodel ped_wall_sfm(initial_states);

    for (int i = 0; i < step_num; i++) {
        states_steps.push_back(ped_wall_sfm.socialforce_simulator());
    }

    drawSFM ped3(states_steps);
    ped3.init_glut("Two pedestrains", 500, 500, 8, 5, 200, 200);
    ped3.draw();
    glutMainLoop();
}


void ped3_crossing() {
    int step_num = 60;
    std::vector<std::vector<double>> initial_states{
        { 0.0, -5.0, 0.5, 0.0, 0.0, 5.0},
        { 0.6, 5.0, -0.5, 0.0, 0.6, -5.0},
        { 2.0, 5.0, -0.5, 0.0, -2.0, -5.0}
    };
    std::vector<std::vector<double>> temp_states;
    std::vector<std::vector<std::vector<double>>> states_steps;

    socialforcemodel ped_wall_sfm(initial_states);

    for (int i = 0; i < step_num; i++) {
        states_steps.push_back(ped_wall_sfm.socialforce_simulator());
    }

    drawSFM ped3(states_steps);
    ped3.init_glut("Three pedestrains", 500, 500, 8, 5, 200, 200);
    ped3.draw();
    glutMainLoop();
}

void ped_wall_crossing() {
    int step_num = 75;
    int space_size = 50;
    double space_begin = -1.0, space_end = 4.0, space_dis;

    std::cout.precision(10);
    std::vector<std::vector<double>> spaces(space_size);

    std::vector<std::vector<double>> initial_states{
        {-10.0, -0.0, 1.0, 0.0, 10.0, 0.0 }
    };
    std::vector<std::vector<double>> temp_states;
    std::vector<std::vector<std::vector<double>>> states_steps;


    // space
    space_dis = (space_end - space_begin) / (float(space_size) - 1.0);
    for (int i = 0; i < space_size; i++) {
        spaces[i].push_back(space_begin);
        spaces[i].push_back(space_begin);
        space_begin += space_dis;
        //std::cout << spaces[i][0] << " " << spaces[i][1] << "\n";
    }

    socialforcemodel ped_wall_sfm(initial_states, spaces);

    for (int i = 0; i < step_num; i++) {
        states_steps.push_back(ped_wall_sfm.socialforce_simulator());
    }

    std::vector<std::vector<std::vector<double>>> step_spaces;
    step_spaces.push_back(spaces);
    drawSFM ped_wall(states_steps, step_spaces);
    ped_wall.init_glut("A pedestrain cross a wall", 500, 500, 8, 5, 200, 200);
    ped_wall.draw();
    glutMainLoop();
}

void peds_wall_crossing() {

    int step_num = 75;
    int space_size = 50;
    double space_begin = -1.0, space_end = 4.0, space_dis;

    std::cout.precision(10);
    std::vector<std::vector<double>> spaces(space_size);

    std::vector<std::vector<double>> initial_states{
        {-9.0, -0.0, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -1.5, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -2.0, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -2.5, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -3.0, 1.0, 0.0, 10.0, 0.0},
        {10.0, 1.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 2.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 3.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 4.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 5.0, -1.0, 0.0, -10.0, 0.0},
    };
    std::vector<std::vector<double>> temp_states;
    std::vector<std::vector<std::vector<double>>> states_steps;


    // space
    space_dis = (space_end - space_begin) / (float(space_size) - 1.0);
    for (int i = 0; i < space_size; i++) {
        spaces[i].push_back(space_begin);
        spaces[i].push_back(space_begin);
        space_begin += space_dis;
        //std::cout << spaces[i][0] << " " << spaces[i][1] << "\n";
    }

    socialforcemodel ped_wall_sfm(initial_states, spaces);

    for (int i = 0; i < step_num; i++) {
        states_steps.push_back(ped_wall_sfm.socialforce_simulator());
    }

    std::vector<std::vector<std::vector<double>>> step_spaces;
    step_spaces.push_back(spaces);
    drawSFM ped_wall(states_steps, step_spaces);
    ped_wall.init_glut("A pedestrain cross a wall", 500, 500, 8, 5, 200, 200);
    ped_wall.draw();
    glutMainLoop();
}


void peds_gate_crossing() {
    int step_num = 150;
    int space_size = 1000;

    std::cout.precision(10);
    std::vector<std::vector<double>> spaces(space_size*2);
    std::vector<std::vector<std::vector<double>>> step_spaces(2, std::vector<std::vector<double>>(space_size));
    std::vector<std::vector<double>> initial_states{
        {-9.0, -0.0, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -1.5, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -2.0, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -2.5, 1.0, 0.0, 10.0, 0.0},
        {-10.0, -3.0, 1.0, 0.0, 10.0, 0.0},
        {10.0, 1.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 2.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 3.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 4.0, -1.0, 0.0, -10.0, 0.0},
        {10.0, 5.0, -1.0, 0.0, -10.0, 0.0},
    };
    std::vector<std::vector<double>> temp_states;
    std::vector<std::vector<std::vector<double>>> states_steps;


    // space
    double space_begin = -10.0, space_end = -0.7, space_dis;
    space_dis = (space_end - space_begin) / (float(space_size) - 1.0);
    for (int i = 0; i < space_size; i++) {
        spaces[i].push_back(0);
        step_spaces[0][i].push_back(0);
        spaces[i].push_back(space_begin);
        step_spaces[0][i].push_back(space_begin);
        space_begin += space_dis;
        //std::cout << spaces[i][0] << " " << spaces[i][1] << "\n";
    }
    space_begin = 10.0, space_end = 0.7;
    space_dis = (space_end - space_begin) / (float(space_size) - 1.0);
    
    for (int i = space_size; i < space_size*2; i++) {
        spaces[i].push_back(0);
        step_spaces[1][i- space_size].push_back(0);
        spaces[i].push_back(space_begin);
        step_spaces[1][i-space_size].push_back(space_begin);
        space_begin += space_dis;
        //std::cout << spaces[i][0] << " " << spaces[i][1] << "\n";
    }

    socialforcemodel ped_wall_sfm(initial_states, spaces);

    for (int i = 0; i < step_num; i++) {
        states_steps.push_back(ped_wall_sfm.socialforce_simulator());
    }
    
    //step_spaces.push_back(spaces);
    drawSFM ped_wall(states_steps, step_spaces);
    ped_wall.init_glut("A pedestrain cross a wall", 500, 500, 8, 5, 200, 200);
    ped_wall.draw();
    glutMainLoop();
}


void peds_aisle_crossing_60() {
    int step_num = 100;
    int space_size = 1000;

    std::cout.precision(10);
    std::vector<std::vector<double>> spaces(space_size * 2);
    std::vector<std::vector<std::vector<double>>> step_spaces(2, std::vector<std::vector<double>>(space_size));
    std::vector<std::vector<double>> initial_states{
        {9.952188804675181, 2.293282467619549, 1.3783455302141503, 0.0, 100.0, 0.0},
        {-6.200988836500367, 0.9757231320484211, 1.1893813477572037, 0.0, 100.0, 0.0},
        {9.460687166512248, -0.4057421694918073, 1.4671437024240088, 0.0, 100.0, 0.0},
        {-6.832510583937319, 1.7783101475380214, 1.51607413234393, 0.0, 100.0, 0.0},
        {0.05168694009235919, 2.210751207008746, 1.4684480235301278, 0.0, 100.0, 0.0},
        {6.26432730503232, 0.6212826149200684, 1.2842566312995665, 0.0, 100.0, 0.0},
        {-3.189753331198417, -1.4017876346639557, 0.8311475126553096, 0.0, 100.0, 0.0},
        {-6.07868745538447, -0.912028207400411, 1.2570777401030213, 0.0, 100.0, 0.0},
        {2.798629905924026, 0.22486553848518387, 1.1936402825803052, 0.0, 100.0, 0.0},
        {1.2017915595723805, 1.131004882214564, 0.8203810586702526, 0.0, 100.0, 0.0},
        {2.730518803278663, -2.155585696977218, 1.0176743411908467, 0.0, 100.0, 0.0},
        {7.055960144246067, 1.183502220109407, 2.016805411825534, 0.0, 100.0, 0.0},
        {0.39704909156877966, 1.5036062204059442, 1.397624314991458, 0.0, 100.0, 0.0},
        {-1.1153726195837543, 1.2885109895847764, 1.3592133193409885, 0.0, 100.0, 0.0},
        {-8.126896680922389, 0.5856395701474759, 1.0948620816512238, 0.0, 100.0, 0.0},
        {-8.369853071885856, 0.30864118283905617, 1.2682740175618483, 0.0, 100.0, 0.0},
        {-5.5529845103176605, 2.1257189546761572, 1.2867398521345526, 0.0, 100.0, 0.0},
        {9.05710704071553, 0.9993107965112796, 1.1074213127097094, 0.0, 100.0, 0.0},
        {5.954845393378747, 0.9266777714396629, 0.9724922003497818, 0.0, 100.0, 0.0},
        {6.839810949531985, -1.4956405183403914, 0.9827728622664834, 0.0, 100.0, 0.0},
        {-0.1448041182305726, -2.043327694965213, 1.2536229957571923, 0.0, 100.0, 0.0},
        {-3.661731434180129, -1.1373087202471033, 1.4081789670624638, 0.0, 100.0, 0.0},
        {4.663437298714945, -0.596691781766921, 1.359660017980524, 0.0, 100.0, 0.0},
        {3.326612140048284, -2.2111446133239343, 1.7641275201999185, 0.0, 100.0, 0.0},
        {0.11272475517398073, 1.97239855394983, 1.1634424077931866, 0.0, 100.0, 0.0},
        {6.9948326435510815, 0.9136216555079713, 1.5409433112443904, 0.0, 100.0, 0.0},
        {2.1867228609586364, -1.1417384518748501, 0.9995124847045265, 0.0, 100.0, 0.0},
        {-8.805345125722182, -0.1340361199263751, 1.2867601491226197, 0.0, 100.0, 0.0},
        {-3.4387391361142305, 0.38429003475470846, 1.4873240920023347, 0.0, 100.0, 0.0},
        {4.445851103806477, -0.7784211013259662, 1.339387437989147, 0.0, 100.0, 0.0},
        {7.68810502235927, -1.4042598568660725, -1.3272711199927654, 0.0, -100.0, 0.0},
        {-0.9681730593203786, 0.6870516890786255, -1.377889766190781, 0.0, -100.0, 0.0},
        {-5.558961532532416, 1.3831837273578311, -1.3429539160198372, 0.0, -100.0, 0.0},
        {-4.437322256648866, 1.860728556284252, -1.3435234291487053, 0.0, -100.0, 0.0},
        {-4.772416319772487, -1.7518898989702008, -1.0142369423562494, 0.0, -100.0, 0.0},
        {-1.9297806790201943, 0.005041797879304366, -1.2173815348047556, 0.0, -100.0, 0.0},
        {7.361294791227381, 1.2301696496647185, -1.3883102436177293, 0.0, -100.0, 0.0},
        {9.657742890614553, 1.0040636941808452, -1.1448937399453416, 0.0, -100.0, 0.0},
        {0.6853553100004994, -2.2811193721301515, -0.9945639113164583, 0.0, -100.0, 0.0},
        {2.4045526770724646, -0.09486464904235214, -1.1522578483256791, 0.0, -100.0, 0.0},
        {6.954144073381756, -0.7867329558321884, -0.5258996249869994, 0.0, -100.0, 0.0},
        {-6.385882171748141, -2.1483931075039258, -1.3045630952685947, 0.0, -100.0, 0.0},
        {2.9175553147999467, -1.228226633288327, -1.677828626862818, 0.0, -100.0, 0.0},
        {9.344098692611917, 1.4379259047502924, -1.0029681994604502, 0.0, -100.0, 0.0},
        {4.3623975546463845, -1.2579240100205165, -1.42219512171127, 0.0, -100.0, 0.0},
        {-1.1761761554276462, -1.9351100335497617, -1.0663819521559204, 0.0, -100.0, 0.0},
        {-0.7442027715242561, 1.3016148136196022, -1.4506148559832301, 0.0, -100.0, 0.0},
        {-1.9340599841804718, -1.4176747270866814, -1.7432107346463963, 0.0, -100.0, 0.0},
        {-0.8914503941458229, 2.499121560548536, -1.7667678896782482, 0.0, -100.0, 0.0},
        {3.2333720746886208, 0.3049213017435659, -1.7029031762684914, 0.0, -100.0, 0.0},
        {6.5687423720714495, -1.346902069471756, -1.3223245462192814, 0.0, -100.0, 0.0},
        {-9.711215705539884, 2.019245422438609, -1.2080114868681564, 0.0, -100.0, 0.0},
        {-0.7019157918938412, 0.7732611842299564, -1.382694817206151, 0.0, -100.0, 0.0},
        {3.101770602642626, -1.2771162164144139, -1.2737797954535441, 0.0, -100.0, 0.0},
        {-3.3874898216433635, 2.168447454790278, -0.9537786310585722, 0.0, -100.0, 0.0},
        {3.9167742236264957, -1.486066803962716, -1.7704892319991614, 0.0, -100.0, 0.0},
        {6.17101963140259, -0.1946272788129072, -1.6388884248555389, 0.0, -100.0, 0.0},
        {-3.955192890131616, 2.1766598244778046, -1.1653160955173487, 0.0, -100.0, 0.0},
        {1.068912434027811, -1.221505658907347, -0.9687421944829467, 0.0, -100.0, 0.0},
        {-4.340257621882689, -2.0558102106869085, -1.6478609066177756, 0.0, -100.0, 0.0},
    };
    std::vector<std::vector<double>> temp_states;
    std::vector<std::vector<std::vector<double>>> states_steps;


    // space
    double space_begin = -10.0, space_end = 10.0, space_dis;
    space_dis = (space_end - space_begin) / (float(space_size) - 1.0);
    for (int i = 0; i < space_size; i++) {
        spaces[i].push_back(space_begin);
        step_spaces[0][i].push_back(space_begin);
        spaces[i].push_back(2.5);
        step_spaces[0][i].push_back(2.5);
        space_begin += space_dis;
        //std::cout << spaces[i][0] << " " << spaces[i][1] << "\n";
    }
    space_begin = -10.0, space_end = 10.0;
    space_dis = (space_end - space_begin) / (float(space_size) - 1.0);
    for (int i = space_size; i < space_size * 2; i++) {
        
        spaces[i].push_back(space_begin);
        step_spaces[1][i - space_size].push_back(space_begin);
        spaces[i].push_back(-2.5);
        step_spaces[1][i - space_size].push_back(-2.5);
        space_begin += space_dis;
        //std::cout << spaces[i][0] << " " << spaces[i][1] << "\n";
    }

    socialforcemodel ped_wall_sfm(initial_states, spaces);

    for (int i = 0; i < step_num; i++) {
        states_steps.push_back(ped_wall_sfm.socialforce_simulator());
    }

    //step_spaces.push_back(spaces);
    drawSFM ped_wall(states_steps, step_spaces);
    ped_wall.init_glut("A pedestrain cross a wall", 500, 500, 8, 5, 200, 200);
    ped_wall.draw();
    glutMainLoop();
}

