// header: draw the result of social force model by using opengl
#include <GL/glut.h>
#include <Windows.h>
class drawSFM {
public:
    
    drawSFM(std::vector<std::vector<std::vector<double>>> steps): m_states_steps(steps), m_max_steps(steps.size()) {};
    drawSFM(std::vector<std::vector<std::vector<double>>> steps, std::vector<std::vector<std::vector<double>>> spaces)
        : m_states_steps(steps), m_max_steps(steps.size()), m_spaces(spaces) {};

    void init_glut(const char*, int, int, float, float, int, int);
    void draw();
    void point_mode();
    void endLoop();


private:
    std::vector<std::vector<std::vector<double>>> m_states_steps;
    std::vector<std::vector<std::vector<double>>> m_spaces;

    int m_windows_width = 500, m_windows_height = 500;
    int m_step_num = 0, m_max_steps;
    void step_idle(); // idle function for deform and rotate
    

};

// function: idle function to control the steps
void drawSFM::step_idle() {

    Sleep(100);
    
    m_step_num++;
    m_step_num %= m_max_steps;

    glutPostRedisplay();
}

// function: points mMode
void drawSFM::point_mode() {


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    step_idle();
    unsigned int i, points_num;

    if (m_spaces.size() != 0) {
        for (i = 0; i < m_spaces.size(); i++) {
            points_num = m_spaces[i].size() - 1;
            glBegin(GL_LINES);
            for (int j = 0; j < points_num; j++) {
                glVertex2d(m_spaces[i][j][0] / 10, m_spaces[i][j][1] / 10);
                glVertex2d(m_spaces[i][j + 1][0] / 10, m_spaces[i][j + 1][1] / 10);
            }
            glEnd();
        }
    }

    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);
    for (i = 0; i < m_states_steps[i].size(); i++) {
        if (i == m_states_steps[i].size() / 2) {
            glColor3f(0.0, 0.0, 1.0);
        }

        glVertex2d(m_states_steps[m_step_num][i][0]/10, m_states_steps[m_step_num][i][1]/10);
        //std::cout << m_states_steps[m_step_num][i][0] << " " << m_states_steps[m_step_num][i][1] << "\n";        
    }
    glColor3f(0.0, 0.0, 0.0);
    glEnd();
    glPopMatrix();
    glFlush();

}

drawSFM* disrefer;
static void call_point_mode() { disrefer->point_mode(); }

// function: draw the object
void drawSFM::draw() {

    disrefer = this;

    glutDisplayFunc(::call_point_mode);
    //glutIdleFunc(::call_point_mode);

    //glutMainLoop();

}

void drawSFM::endLoop() {
    glutMainLoop();
}

// function: initialize the glut windows
void drawSFM::init_glut(const char* name, int w_width, int w_height, float point_size = 8.0, float line_width = 5.0, int win_x = 50, int win_y = 50) {

    //glutInit(&argc, argv);

    m_windows_width = w_width;
    m_windows_height = w_height;

    glutInitDisplayMode(GLUT_SINGLE | GLUT_DEPTH);
    glutInitWindowSize(m_windows_width, m_windows_height);
    glutInitWindowPosition(win_x, win_y);
    glutCreateWindow(name);

    glEnable(GL_DEPTH_TEST);
    glPointSize(point_size);
    glLineWidth(line_width);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glColor3f(0.0, 0.0, 0.0);
}