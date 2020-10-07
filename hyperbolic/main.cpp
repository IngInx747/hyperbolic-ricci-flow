#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef MAC_OS
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif // MAC_OS

#include "viewer/Arcball.h" /*  Arc Ball  Interface         */
#include "HyperbolicMesh.h"
#include "HyperbolicMap.h"

using namespace MeshLib;
using M = CHyperbolicMesh;

/* window width and height */
int g_win_width, g_win_height;
int g_button;
int g_startx, g_starty;
int g_shade_flag = 0;
bool g_show_domn = true;
bool g_show_open = true;
bool g_show_orgn = false;
bool g_show_axis = false;
bool g_show_bndr = true;
bool g_show_circ = true;

/* rotation quaternion and translation vector for the object */
CQrot g_obj_rot(1, 0, 0, 0); // 0 0 1 0
CPoint g_obj_trans(0, 0, 0);

/* arcball object */
CArcball g_arcball;

/* global g_mesh */
CHyperbolicMesh g_mesh;
HyperbolicMap g_hyperbolic_map;

void set_color(int color)
{
    if (!color) return;

    color = color > 0 ? color : -color;
    color = ((color - 1) % 9) + 1;

    switch (color)
    {
    case 1: glColor3f(1.0f, 0.0f, 0.0f); break;
    case 2: glColor3f(0.0f, 1.0f, 0.0f); break;
    case 3: glColor3f(0.0f, 0.0f, 1.0f); break;
    case 4: glColor3f(1.0f, 1.0f, 0.0f); break;
    case 5: glColor3f(0.0f, 1.0f, 1.0f); break;
    case 6: glColor3f(1.0f, 0.0f, 1.0f); break;
    case 7: glColor3f(1.0f, 1.0f, 1.0f); break;
    case 8: glColor3f(0.2f, 0.2f, 0.2f); break;
    case 9: glColor3f(0.5f, 0.5f, 0.5f); break;
    default: break;
    }
}

void drawAxis()
{
    glPushMatrix();
    // move the axes to the screen corner
    glTranslatef(0.0, 0.0, 0.0);
    // draw our axes
    glDisable(GL_LIGHTING);
    glLineWidth(5.0);
    glBegin(GL_LINES);
    // draw line for x axis
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    // draw line for y axis
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    // draw line for Z axis
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();
    // load the previous matrix
    glPopMatrix();
}

void drawUnitBox()
{
    glPushMatrix();
    // move the axes to the screen corner
    glTranslatef(0.0, 0.0, 0.0);
    // draw our axes
    glDisable(GL_LIGHTING);
    glLineWidth(1.0);
    glBegin(GL_LINES);
    glColor3f(1.0, 1.0, 1.0);
    // draw lines
    glVertex3f(-1.0, -1.0, -1.0); glVertex3f( 1.0, -1.0, -1.0);
    glVertex3f(-1.0,  1.0, -1.0); glVertex3f( 1.0,  1.0, -1.0);
    glVertex3f(-1.0, -1.0,  1.0); glVertex3f( 1.0, -1.0,  1.0);
    glVertex3f(-1.0,  1.0,  1.0); glVertex3f( 1.0,  1.0,  1.0);
    glVertex3f(-1.0, -1.0, -1.0); glVertex3f(-1.0,  1.0, -1.0);
    glVertex3f( 1.0, -1.0, -1.0); glVertex3f( 1.0,  1.0, -1.0);
    glVertex3f(-1.0, -1.0,  1.0); glVertex3f(-1.0,  1.0,  1.0);
    glVertex3f( 1.0, -1.0,  1.0); glVertex3f( 1.0,  1.0,  1.0);
    glVertex3f(-1.0, -1.0, -1.0); glVertex3f(-1.0, -1.0,  1.0);
    glVertex3f( 1.0, -1.0, -1.0); glVertex3f( 1.0, -1.0,  1.0);
    glVertex3f(-1.0,  1.0, -1.0); glVertex3f(-1.0,  1.0,  1.0);
    glVertex3f( 1.0,  1.0, -1.0); glVertex3f( 1.0,  1.0,  1.0);
    glEnd();
    // load the previous matrix
    glPopMatrix();
}

/*! setup the object, transform from the world to the object coordinate system */
void setupObject(void)
{
    double rot[16];

    glTranslated(g_obj_trans[0], g_obj_trans[1], g_obj_trans[2]);
    g_obj_rot.convert(rot);
    glMultMatrixd((GLdouble*) rot);
}

/*! the eye is always fixed at world z = +5 */
void setupEye(void)
{
    glLoadIdentity();
    gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0);
}

/*! setup light */
void setupLight()
{
    GLfloat lightOnePosition[4] = {0, 0, 1, 0};
    GLfloat lightTwoPosition[4] = {0, 0, -1, 0};
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);
}

/*! draw g_mesh */
void drawMesh(M* pMesh, int id = 0)
{
    glEnable(GL_LIGHTING);
    glLineWidth(1.0);
    glColor3f(153.0 / 255.0, 204.0 / 255.0, 255.0 / 255.0);
    set_color(id);

    for (M::MeshFaceIterator fiter(pMesh); !fiter.end(); ++fiter)
    {
        glBegin(GL_POLYGON);
        M::CFace* pF = *fiter;
        for (M::FaceVertexIterator fviter(pF); !fviter.end(); ++fviter)
        {
            M::CVertex* pV = *fviter;
            CPoint& p = pV->point();
            CPoint n;
            switch (g_shade_flag)
            {
                case 0:
                    n = pF->normal();
                    break;
                case 1:
                    n = pV->normal();
                    break;
            }
            glNormal3d(n[0], n[1], n[2]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
    }
}

void drawSharpEdges(M* pMesh)
{
    if (!g_show_bndr) return;

    glDisable(GL_LIGHTING);
    glLineWidth(5.);
    glColor3f(0.0f, 1.0f, 0.0f);

    glBegin(GL_LINES);
    for (M::MeshEdgeIterator eiter(pMesh); !eiter.end(); ++eiter)
    {
        M::CEdge* pE = *eiter;

        if (pE->sharp())
        {
            set_color(pE->sharp());
            M::CVertex* p0 = pMesh->edgeVertex1(pE);
            M::CVertex* p1 = pMesh->edgeVertex2(pE);
            glVertex3f(p0->point()[0], p0->point()[1], p0->point()[2]);
            glVertex3f(p1->point()[0], p1->point()[1], p1->point()[2]);
        }
    }
    glEnd();
}

void drawArch(const CPoint2& center, double radius)
{
    int numSample = 200;
    if (!g_show_circ) return;

    glDisable(GL_LIGHTING);
    glLineWidth(2.);
    glColor3f(0.8f, 0.8f, 0.8f);

    const float pi = 3.14159265359f;
    float dt = (pi * 2) / numSample;

    glBegin(GL_LINES);
    for (int i = 0; i < numSample; i++)
    {
        float t = dt * i;
        float x = radius * cosf(t) + center[0];
        float y = radius * sinf(t) + center[1];
        if (x * x + y * y > 1) continue;
        glVertex3f(x, y, 0.001f);
        x = radius * cosf(t + dt) + center[0];
        y = radius * sinf(t + dt) + center[1];
        glVertex3f(x, y, 0.001f);
    }
    glEnd();
}

/*! display call back function
 */
void display()
{
    /* clear frame buffer */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setupLight();
    /* transform from the eye coordinate system to the world system */
    setupEye();
    glPushMatrix();
    /* transform from the world to the ojbect coordinate system */
    setupObject();

    if (g_show_axis)
    {
        drawAxis();
        drawUnitBox();
    }

    if (g_show_domn)
    {
        drawSharpEdges(&g_hyperbolic_map.domain_mesh());
        drawMesh(&g_hyperbolic_map.domain_mesh());

        for (int i = 0; i < g_hyperbolic_map.tessellation_meshes().size(); ++i)
        {
            auto& mesh = g_hyperbolic_map.tessellation_meshes()[i];
            int id = g_hyperbolic_map.tessellation_index(i);
            drawSharpEdges(&mesh);
            drawMesh(&mesh, id);
        }

        for (auto& p : g_hyperbolic_map.geodesic_circles())
        {
            drawArch(p.first, p.second);
        }
    }

    if (g_show_open)
    {
        drawSharpEdges(&g_hyperbolic_map.open_mesh());
        drawMesh(&g_hyperbolic_map.open_mesh());
    }

    if (g_show_orgn)
    {
        drawSharpEdges(&g_mesh);
        drawMesh(&g_mesh);
    }

    glPopMatrix();
    glutSwapBuffers();
}

/*! Called when a "resize" event is received by the window. */
void reshape(int w, int h)
{
    float ar;
    
    g_win_width = w;
    g_win_height = h;

    ar = (float) (w) / h;
    glViewport(0, 0, w, h); /* Set Viewport */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(40.0, /* field of view in degrees */
                   ar,   /* aspect ratio */
                   0.1,  /* Z near */
                   100.0 /* Z far */);

    glMatrixMode(GL_MODELVIEW);

    glutPostRedisplay();
}

/*! helper function to remind the user about commands, hot keys */
void help()
{
    printf("1  -  Perform Ricci flow on mesh\n");
    printf("2  -  Slice mesh into a fundamental domain\n");
    printf("3  -  Embed domain into Poincare disk\n");
    printf("4  -  Tessellate domains into Poincare disk\n");
    printf("5  -  Tessellate disk, one domain at a time\n");
    printf("6  -  Fine boundary with geodesic lines\n");
    printf("0  -  Clear tesselation\n");
    printf("w  -  Wireframe Display\n");
    printf("f  -  Flat Shading \n");
    printf("s  -  Smooth Shading\n");
    printf("b  -  Show boundary\n");
    printf("o  -  Show open mesh\n");
    printf("m  -  Show original mesh\n");
    printf("x  -  Show xyz axis\n");
    printf("?  -  Help Information\n");
    printf("esc - Quit\n");
}

/*! Keyboard call back function */
void keyBoard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case '1':
            //
            g_hyperbolic_map.ricci_flow(1e-12, 1, 1);
            break;
        case '2':
            // generate a cut graph as fundamental domain
            g_hyperbolic_map.mark_fundamental_domain();
            // slice mesh along the fundamental domain
            g_hyperbolic_map.slice_fundamental_domain();
            g_show_open = true;
            g_show_bndr = true;
            break;
        case '3':
            // embed sliced mesh into Poincare disk
            g_hyperbolic_map.isometric_embed();
            // sort out segments on domain boundary
            g_hyperbolic_map.sort_domain_boundaries();
            // compute fuchsian group
            g_hyperbolic_map.compute_fuchsian_group();
            g_show_domn = true;
            break;
        case '4':
            // tessellate Poincare disk
            g_hyperbolic_map.tessellate_disk(1);
            g_show_domn = true;
            break;
        case '5':
            // tessellate Poincare disk one domain at a time
            g_hyperbolic_map.tessellate_disk_single_step(2);
            g_show_domn = true;
            break;
        case '6':
            // draw geodesic circles
            g_hyperbolic_map.compute_geodesic_cycles();
            g_show_circ = true;
            break;
        case '0':
            // clear tessellation
            g_hyperbolic_map.clear_tessellation();
            break;
        case 'f':
            // Flat Shading
            glPolygonMode(GL_FRONT, GL_FILL);
            g_shade_flag = 0;
            break;
        case 's':
            // Smooth Shading
            glPolygonMode(GL_FRONT, GL_FILL);
            g_shade_flag = 1;
            break;
        case 'w':
            // Wireframe mode
            glPolygonMode(GL_FRONT, GL_LINE);
            break;
        case 'd':
            // Show original mesh
            g_show_domn = !g_show_domn;
            break;
        case 'b':
            // Show original mesh
            g_show_bndr = !g_show_bndr;
            break;
        case 'c':
            // Show geodesic circles
            g_show_circ = !g_show_circ;
            break;
        case 'o':
            // Show open mesh
            g_show_open = !g_show_open;
            break;
        case 'm':
            // Show original mesh
            g_show_orgn = !g_show_orgn;
            break;
        case 'x':
            // Show original mesh
            g_show_axis = !g_show_axis;
            break;
        case '?':
            help();
            break;
        case 27:
            exit(0);
            break;
    }
    glutPostRedisplay();
}

/*! setup GL states */
void setupGLstate()
{
    GLfloat lightOneColor[] = {1, 1, 1, 1.0};
    GLfloat globalAmb[] = {.1, .1, .1, 1};
    GLfloat lightOnePosition[] = {.0, 0.0, 1.0, 1.0};
    GLfloat lightTwoPosition[] = {.0, 0.0, -1.0, 1.0};

    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.35, 0.53, 0.70, 0);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightOneColor);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);

    const GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0f);

    GLfloat mat_ambient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat mat_diffuse[] = {0.01f, 0.01f, 0.01f, 1.0f};
    GLfloat mat_specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat mat_shininess[] = {32};

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

/*! mouse click call back function */
void mouseClick(int button, int state, int x, int y)
{
    /* set up an arcball around the Eye's center
    switch y coordinates to right handed system  */

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        g_button = GLUT_LEFT_BUTTON;
        g_arcball = CArcball(g_win_width, 
                             g_win_height, 
                             x - g_win_width / 2, 
                             g_win_height - y - g_win_height / 2);
    }

    if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
    {
        g_startx = x;
        g_starty = y;
        g_button = GLUT_MIDDLE_BUTTON;
    }

    if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        g_startx = x;
        g_starty = y;
        g_button = GLUT_RIGHT_BUTTON;
    }
    return;
}

/*! mouse motion call back function */
void mouseMove(int x, int y)
{
    CPoint trans;
    CQrot rot;

    /* rotation, call g_arcball */
    if (g_button == GLUT_LEFT_BUTTON)
    {
        rot = g_arcball.update(x - g_win_width / 2, g_win_height - y - g_win_height / 2);
        g_obj_rot = rot * g_obj_rot;
        glutPostRedisplay();
    }

    /*xy translation */
    if (g_button == GLUT_MIDDLE_BUTTON)
    {
        double scale = 10. / g_win_height;
        trans = CPoint(scale * (x - g_startx), scale * (g_starty - y), 0);
        g_startx = x;
        g_starty = y;
        g_obj_trans = g_obj_trans + trans;
        glutPostRedisplay();
    }

    /* zoom in and out */
    if (g_button == GLUT_RIGHT_BUTTON)
    {
        double scale = 10. / g_win_height;
        trans = CPoint(0, 0, scale * (g_starty - y));
        g_startx = x;
        g_starty = y;
        g_obj_trans = g_obj_trans + trans;
        glutPostRedisplay();
    }
}

void initOpenGL(int argc, char* argv[])
{
    /* glut stuff */
    glutInit(&argc, argv); /* Initialize GLUT */
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutCreateWindow("Mesh Viewer"); /* Create window with given title */
    glViewport(0, 0, 600, 600);

    glutDisplayFunc(display); /* Set-up callback functions */
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
    glutKeyboardFunc(keyBoard);
    setupGLstate();

    glutMainLoop(); /* Start GLUT event-processing loop */
}

/*! main function for viewer
 */
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s input.m\n", argv[0]);
        return EXIT_FAILURE;
    }

    std::string mesh_name(argv[1]);
    if (strutil::endsWith(mesh_name, ".m"))
    {
        g_mesh.read_m(mesh_name.c_str());
    }
    else
    {
        printf("Only file format .m supported.\n");
        return EXIT_FAILURE;
    }

    g_mesh.normalize();
    g_mesh.computeNormal();
    g_hyperbolic_map.set_mesh(&g_mesh);
    help();

    initOpenGL(argc, argv);

    return EXIT_SUCCESS;
}
