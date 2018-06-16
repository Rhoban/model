#include <SFML/OpenGL.hpp>
#include "GL/glut.h"
#include "Viewer/ModelViewer.hpp"

namespace Leph {

ModelViewer::ModelViewer(unsigned int width, 
    unsigned int height) :
    _width(width),
    _height(height),
    _window(sf::VideoMode(width, height), "OpenGL", 
        sf::Style::Default, sf::ContextSettings(32)),
    _font(),
    _camPos(0.0, 0.0, 0.0),
    _camView(1.0, 0.0, 0.0),
    _camRadius(1.0),
    _camAngle1(0.0),
    _camAngle2(1.0),
    _camOffsetX(0.0),
    _camOffsetY(0.0),
    _trajectoryRed(),
    _trajectoryGreen(),
    _trajectoryBlue(),
    _trajectoryYellow(),
    _trajectoryPurple(),
    _trajectoryCyan(),
    _lastMousePosX(0),
    _lastMousePosY(0)
{
    //Load font file
    if (!_font.loadFromFile("font.ttf")) {
        throw std::logic_error("ModelViewer fail to load font");
    }

    _window.setVerticalSyncEnabled(true);
    _camView.normalize();
    
    //Setup projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(
        60.0, (double)width/(double)height, 
        0.01, 1000.0);       
    //Enable Z-buffer
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthRange(0.0f, 1.0f);
    //Set color and depth clear value
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClearDepth(1.0f);
}
        
bool ModelViewer::update(bool freeFly)
{
    //Handle keyboard
    sf::Event event;
    double wheelDelta = 0.0;
    while (_window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            return false;
        }
        if (event.type == sf::Event::KeyPressed && 
            event.key.code == sf::Keyboard::Escape
        ) {
            return false;
        }
        if(event.type == sf::Event::MouseWheelMoved) {
            wheelDelta = event.mouseWheel.delta;
        }
    }
    //Handle mouse relative motion
    sf::Vector2i mousePosition = sf::Mouse::getPosition(_window);
    double mouseDeltaPosX = mousePosition.x - _lastMousePosX;
    double mouseDeltaPosY = mousePosition.y - _lastMousePosY;
    _lastMousePosX = mousePosition.x;
    _lastMousePosY = mousePosition.y;
    //Check if the mouse cursor is in the screen
    bool isMouseInScreen = true;
    if (
        mousePosition.x < 0 || 
        mousePosition.y < 0 || 
        mousePosition.x >= (int)_window.getSize().x || 
        mousePosition.y > (int)_window.getSize().y
    ) {
        isMouseInScreen = false;
    }
    //Camera view control
    if (freeFly) {
        Eigen::Vector3d camLat = _camView.cross(
            Eigen::Vector3d(0.0, 0.0, 1.0));
        Eigen::Vector3d camUp = _camView.cross(camLat);
        _camView = Eigen::AngleAxisd(-camViewVel*mouseDeltaPosY, camLat)
            .toRotationMatrix()*_camView;
        _camView = Eigen::AngleAxisd(camViewVel*mouseDeltaPosX, camUp)
            .toRotationMatrix()*_camView;
        _camView.normalize();
        //Camera position control
        if (
            sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || 
            sf::Keyboard::isKeyPressed(sf::Keyboard::Z) 
        ) {
            _camPos += camPosVel*_camView;
        }
        if (
            sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || 
            sf::Keyboard::isKeyPressed(sf::Keyboard::S) 
        ) {
            _camPos -= camPosVel*_camView;
        }
        if (
            sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || 
            sf::Keyboard::isKeyPressed(sf::Keyboard::Q) 
        ) {
            _camPos -= camPosVel*camLat;
        }
        if (
            sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || 
            sf::Keyboard::isKeyPressed(sf::Keyboard::D) 
        ) {
            _camPos += camPosVel*camLat;
        }
    } else {
        if (isMouseInScreen) {
            if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
                _camAngle1 += -0.5*camViewVel*mouseDeltaPosX;
                _camAngle2 += -0.5*camViewVel*mouseDeltaPosY;
            }
            _camRadius += -wheelDelta*camPosVel*20.0;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Subtract)) {
                _camRadius += 0.1;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Add)) {
                _camRadius -= 0.1;
                if (_camRadius <= 0.0) {
                    _camRadius = 0.0;
                }
            }
            //Camera position control
            if (
                sf::Keyboard::isKeyPressed(sf::Keyboard::Up) || 
                sf::Keyboard::isKeyPressed(sf::Keyboard::Z) 
            ) {
                Eigen::Vector3d view = (_camView - _camPos);
                view.normalize();
                _camOffsetX += 5.0*camPosVel*view.x();
                _camOffsetY += 5.0*camPosVel*view.y();
            }
            if (
                sf::Keyboard::isKeyPressed(sf::Keyboard::Down) || 
                sf::Keyboard::isKeyPressed(sf::Keyboard::S) 
            ) {
                Eigen::Vector3d view = (_camView - _camPos);
                view.normalize();
                _camOffsetX -= 5.0*camPosVel*view.x();
                _camOffsetY -= 5.0*camPosVel*view.y();
            }
            if (
                sf::Keyboard::isKeyPressed(sf::Keyboard::Left) || 
                sf::Keyboard::isKeyPressed(sf::Keyboard::Q) 
            ) {
                Eigen::Vector3d view = (_camView - _camPos);
                view.normalize();
                view = view.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
                _camOffsetX -= 5.0*camPosVel*view.x();
                _camOffsetY -= 5.0*camPosVel*view.y();
            }
            if (
                sf::Keyboard::isKeyPressed(sf::Keyboard::Right) || 
                sf::Keyboard::isKeyPressed(sf::Keyboard::D) 
            ) {
                Eigen::Vector3d view = (_camView - _camPos);
                view.normalize();
                view = view.cross(Eigen::Vector3d(0.0, 0.0, 1.0));
                _camOffsetX += 5.0*camPosVel*view.x();
                _camOffsetY += 5.0*camPosVel*view.y();
            }
        }
        if (_camAngle2 < 0.01) _camAngle2 = 0.01;
        if (_camAngle2 > M_PI-0.01) _camAngle2 = M_PI-0.01;
        if (_camRadius <= 0.0) _camRadius += wheelDelta*camPosVel*20.0;
        _camView.x() = _camOffsetX;
        _camView.y() = _camOffsetY;
        _camPos = _camView;
        _camPos.z() += _camRadius*cos(_camAngle2);
        _camPos.x() += _camRadius*cos(_camAngle1)*sin(_camAngle2);
        _camPos.y() += _camRadius*sin(_camAngle1)*sin(_camAngle2);
    }

    //Camera
    updateCamera(freeFly);
    //Drawing
    drawGround(frameLength*20.0);
    
    //Update drawing
    _window.display();
    //Reset drawing
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    return true;
}
        
bool ModelViewer::isKeyPressed(sf::Keyboard::Key key)
{
    return sf::Keyboard::isKeyPressed(key); 
}
        
void ModelViewer::drawFrame(
    const Eigen::Vector3d& center, 
    const Eigen::Matrix3d& orientation)
{
    glPushMatrix();
    glTranslated(center.x(), center.y(), center.z());
    applyRotation(orientation.transpose());
    glLineWidth(frameThickness);
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(frameLength, 0.0, 0.0);
    glEnd();
    glPushMatrix();
        glTranslated(frameLength, 0.0, 0.0);
        glScaled(frameLength/20.0, frameLength/20.0, frameLength/20.0);
        drawCube(1.0, 0.0, 0.0);
    glPopMatrix();
    glBegin(GL_LINES);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, frameLength, 0.0);
    glEnd();
    glPushMatrix();
        glTranslated(0.0, frameLength, 0.0);
        glScaled(frameLength/20.0, frameLength/20.0, frameLength/20.0);
        drawCube(0.0, 1.0, 0.0);
    glPopMatrix();
    glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, frameLength);
    glEnd();
    glPushMatrix();
        glTranslated(0.0, 0.0, frameLength);
        glScaled(frameLength/20.0, frameLength/20.0, frameLength/20.0);
        drawCube(0.0, 0.0, 1.0);
    glPopMatrix();
    glLineWidth(frameThickness/4.0);
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(-frameLength, 0.0, 0.0);
    glEnd();
    glBegin(GL_LINES);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, -frameLength, 0.0);
    glEnd();
    glBegin(GL_LINES);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, -frameLength);
    glEnd();
    glPopMatrix();
}
        
void ModelViewer::drawMass(const Eigen::Vector3d& center,
    const Eigen::Matrix3d& orientation,
    double color)
{
    glPushMatrix();
        glTranslated(center.x(), center.y(), center.z());
        applyRotation(orientation.transpose());
        glScaled(frameLength/4.0, frameLength/4.0, frameLength/4.0);
        drawCube(0.6*color, 0.3*color, 0.3*color);
    glPopMatrix();
}
        
void ModelViewer::drawJoint(
    const Eigen::Vector3d& center, 
    const Eigen::Matrix3d& orientation,
    double color)
{
    glPushMatrix();
        glTranslated(center.x(), center.y(), center.z());
        applyRotation(orientation.transpose());
        glLineWidth(frameThickness);
        glBegin(GL_LINES);
            glColor3f(0.3*color, 0.3*color, 0.6*color);
            glVertex3f(-2.0*frameLength, 0.0, 0.0);
            glVertex3f(2.0*frameLength, 0.0, 0.0);
        glEnd();
        glLineWidth(20.0*frameThickness);
        glBegin(GL_LINES);
            glColor3f(0.6*color, 0.6*color, 0.8*color);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(1.5*frameLength, 0.0, 0.0);
        glEnd();
        glScaled(3.0*frameLength/4.0, frameLength/6.0, frameLength/6.0);
        drawCube(0.3*color, 0.3*color, 0.6*color);
    glPopMatrix();
}
        
void ModelViewer::drawLink(
    const Eigen::Vector3d& pt1,
    const Eigen::Vector3d& pt2,
    double color)
{
    double dist = (pt1-pt2).norm();
    Eigen::Vector3d vect = pt2 - pt1;

    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    Eigen::Vector3d axis = vect.cross(Eigen::Vector3d(1.0, 0.0, 0.0));
    if (axis.norm() > 0.001) {
        double angle = vect.dot(Eigen::Vector3d(1.0, 0.0, 0.0))/(vect.norm());
        angle = acos(angle);
        axis.normalize();
        transform = Eigen::AngleAxisd(-angle, axis).toRotationMatrix();
    } else if (vect.x() < 0.0) {
        transform = -transform;
    }

    glPushMatrix();
        glTranslated(pt1.x(), pt1.y(), pt1.z());
        applyRotation(transform);
        glScaled(dist/2.0, frameLength/10.0, frameLength/10.0);
        glTranslated(1.0, 0.0, 0.0);
        drawCube(0.3*color, 0.6*color, 0.3*color);
    glPopMatrix();
}

void ModelViewer::drawBox(double sizeX, double sizeY, double sizeZ,
    const Eigen::Vector3d& center, 
    const Eigen::Matrix3d& orientation,
    double r, double g, double b)
{
    glPushMatrix();
        glTranslated(center.x(), center.y(), center.z());
        applyRotation(orientation.transpose());
        glScaled(sizeX, sizeY, sizeZ);
        glLineWidth(frameThickness);
        drawCube(r, g, b, true);
    glPopMatrix();
}
        
void ModelViewer::drawSphere(
    const Eigen::Vector3d& center, double radius,
    double r, double g, double b)
{
    glPushMatrix();
        glLineWidth(3.0*groundThickness);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3d(r, g, b);
        glTranslated(center.x(), center.y(), center.z());
        GLUquadric* quad;
        quad = gluNewQuadric();
        gluSphere(quad, radius, 10, 10);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();
}

void ModelViewer::drawCylinder(
    const Eigen::Vector3d& base, 
    double radius, double height,
    double r, double g, double b)
{
    glPushMatrix();
        glLineWidth(3.0*groundThickness);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor3d(r, g, b);
        glTranslated(base.x(), base.y(), base.z());
        GLUquadric* quad;
        quad = gluNewQuadric();
        gluCylinder(quad, radius, radius, height, 10, 10);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPopMatrix();
}

void ModelViewer::drawArrow(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& vect,
    double length,
    double r, double g, double b)
{
    if (fabs(length) < 0.0001) {
        return;
    }
    glPushMatrix();
        Eigen::Vector3d end = center + length*vect.normalized();
        Eigen::Vector3d sideVect1 = 
            Eigen::AngleAxisd(M_PI/4.0, 
                Eigen::Vector3d(0.0, 0.0, 1.0)).toRotationMatrix()
            * vect.normalized();
        Eigen::Vector3d sideVect2 = 
            Eigen::AngleAxisd(-M_PI/4.0, 
                Eigen::Vector3d(0.0, 0.0, 1.0)).toRotationMatrix()
            * vect.normalized();
        Eigen::Vector3d side1 = end - 0.2*length*sideVect1;
        Eigen::Vector3d side2 = end - 0.2*length*sideVect2;
        glLineWidth(20.0*groundThickness);
        glColor3d(r, g, b);
        glBegin(GL_LINES);
            glVertex3d(center.x(), center.y(), center.z());
            glVertex3d(end.x(), end.y(), end.z());
            glVertex3d(end.x(), end.y(), end.z());
            glVertex3d(side1.x(), side1.y(), side1.z());
            glVertex3d(end.x(), end.y(), end.z());
            glVertex3d(side2.x(), side2.y(), side2.z());
        glEnd();
    glPopMatrix();
}

void ModelViewer::drawLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double thick,
    double r, double g, double b)
{
    glPushMatrix();
        glLineWidth(thick*groundThickness);
        glColor3d(r, g, b);
        glBegin(GL_LINES);
            glVertex3d(start.x(), start.y(), start.z());
            glVertex3d(end.x(), end.y(), end.z());
        glEnd();
    glPopMatrix();
}

void ModelViewer::drawText(
    const Eigen::Vector3d& position,
    unsigned int size, 
    const std::string& str,
    double r, double g, double b)
{
    //Compute 3d position on screen
    Eigen::Vector2d pixel = getPointProjection(position);
    if (
        pixel.x() < 0.0 || pixel.x() >= _width ||
        pixel.y() < 0.0 || pixel.y() >= _height
    ) {
        return;
    }

    //Save all OpenGL state
    _window.pushGLStates();

    //Configure the text
    sf::Text text;
    //Font
    text.setFont(_font); 
    //Text
    text.setString(str);
    //Size in pixel
    text.setCharacterSize(size);
    //Color
    // text.setFillColor(sf::Color(r*255, g*255, b*255));
    //Position
    text.move(pixel.x(), pixel.y());

    //Draw in screen
    _window.draw(text);

    //Restore all OpenGL state
    _window.popGLStates();
}
        
void ModelViewer::addTrackedPoint(const Eigen::Vector3d& point, 
    Color color)
{
    if (color == Red) {
        _trajectoryRed.push_front(point);
        while (_trajectoryRed.size() > maxTrajectory) {
            _trajectoryRed.pop_back();
        }
    }
    if (color == Green) {
        _trajectoryGreen.push_front(point);
        while (_trajectoryGreen.size() > maxTrajectory) {
            _trajectoryGreen.pop_back();
        }
    }
    if (color == Blue) {
        _trajectoryBlue.push_front(point);
        while (_trajectoryBlue.size() > maxTrajectory) {
            _trajectoryBlue.pop_back();
        }
    }
    if (color == Yellow) {
        _trajectoryYellow.push_front(point);
        while (_trajectoryYellow.size() > maxTrajectory) {
            _trajectoryYellow.pop_back();
        }
    }
    if (color == Purple) {
        _trajectoryPurple.push_front(point);
        while (_trajectoryPurple.size() > maxTrajectory) {
            _trajectoryPurple.pop_back();
        }
    }
    if (color == Cyan) {
        _trajectoryCyan.push_front(point);
        while (_trajectoryCyan.size() > maxTrajectory) {
            _trajectoryCyan.pop_back();
        }
    }
}
void ModelViewer::drawTrajectory()
{
    drawColorTrajectory(_trajectoryRed, 1.0, 0.0, 0.0);
    drawColorTrajectory(_trajectoryGreen, 0.0, 1.0, 0.0);
    drawColorTrajectory(_trajectoryBlue, 0.0, 0.0, 1.0);
    drawColorTrajectory(_trajectoryYellow, 1.0, 1.0, 0.0);
    drawColorTrajectory(_trajectoryPurple, 1.0, 0.0, 1.0);
    drawColorTrajectory(_trajectoryCyan, 0.0, 1.0, 1.0);
}
        
void ModelViewer::setCamPosition(double x, double y)
{
    _camOffsetX = x;
    _camOffsetY = y;
}

void ModelViewer::updateCamera(bool useViewVector)
{
    Eigen::Vector3d viewPoint;
    if (useViewVector) {
        viewPoint = _camPos + _camView;
    } else {
        viewPoint = _camView;
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        _camPos.x(), _camPos.y(), _camPos.z(), 
        viewPoint.x(), viewPoint.y(), viewPoint.z(), 
        0.0, 0.0, 1.0);
}
        
void ModelViewer::applyRotation(const Eigen::Matrix3d& orientation)
{
    GLfloat m[16];
    m[0] = orientation(0, 0);
    m[1] = orientation(1, 0);
    m[2] = orientation(2, 0);
    m[3] = 0.0;
    m[4] = orientation(0, 1);
    m[5] = orientation(1, 1);
    m[6] = orientation(2, 1);
    m[7] = 0.0;
    m[8] = orientation(0, 2);
    m[9] = orientation(1, 2);
    m[10] = orientation(2, 2);
    m[11] = 0.0;
    m[12] = 0.0;
    m[13] = 0.0;
    m[14] = 0.0;
    m[15] = 1.0;
    glMultMatrixf(m);
}
        
void ModelViewer::drawGround(double size)
{
    glLineWidth(groundThickness);
    for (double x=-size;x<=size;x+=2.0*frameLength) {
        glBegin(GL_LINES);
            glColor3f(1.0, 1.0, 1.0);
            glVertex3f(x, -size, 0.0);
            glVertex3f(x, size, 0.0);
        glEnd();
    }
    for (double y=-size;y<=size;y+=2.0*frameLength) {
        glBegin(GL_LINES);
            glColor3f(1.0, 1.0, 1.0);
            glVertex3f(-size, y, 0.0);
            glVertex3f(size, y, 0.0);
        glEnd();
    }
}
        
void ModelViewer::drawCube(float r, float g, float b,
    bool isWireFrame)
{
    if (isWireFrame) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    glBegin(GL_QUADS);
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(1.0, 1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
        glVertex3f(1.0, -1.0, 1.0);
        
        glColor3f(r, g, b);
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glVertex3f(-1.0, -1.0, 1.0);
        
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(-1.0, -1.0, 1.0);
        glVertex3f(1.0, -1.0, 1.0);
        
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, -1.0);
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
        
        glColor3f(r, g, b);
        glVertex3f(1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, 1.0);
        glVertex3f(-1.0, 1.0, -1.0);
        glVertex3f(1.0, 1.0, -1.0);

        glColor3f(r, g, b);
        glVertex3f(1.0, -1.0, 1.0);
        glVertex3f(-1.0, -1.0, 1.0);
        glVertex3f(-1.0, -1.0, -1.0);
        glVertex3f(1.0, -1.0, -1.0);
    glEnd();
    if (isWireFrame) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}
        
void ModelViewer::drawColorTrajectory(
    const std::list<Eigen::Vector3d>& traj, 
    double r, double g, double b)
{
    if (traj.size() == 0) {
        return;
    }

    //Draw first point as big point
    glPointSize(frameThickness*3.0);
    glBegin(GL_POINTS);
        glColor3f(r, g, b);
        glVertex3f(
            traj.front().x(), 
            traj.front().y(), 
            traj.front().z());
    glEnd();

    //Draw remaining trajectory as lines
    Eigen::Vector3d oldPt = traj.front();
    size_t length = 1;
    for (const Eigen::Vector3d& pt : traj) {
        double rr = r - (double)length/maxTrajectory;
        double gg = g - (double)length/maxTrajectory;
        double bb = b - (double)length/maxTrajectory;
        if (rr <= 0.0) rr = 0.0;
        if (gg <= 0.0) gg = 0.0;
        if (bb <= 0.0) bb = 0.0;
        if (rr == 0.0 && gg == 0.0 && bb == 0.0) break;
        glLineWidth(frameThickness*1.0);
        glBegin(GL_LINES);
            glColor3f(rr, gg, bb);
            glVertex3f(oldPt.x(), oldPt.y(), oldPt.z());
            glVertex3f(pt.x(), pt.y(), pt.z());
        glEnd();
        oldPt = pt;
        length++;
    }
}

Eigen::Vector2d ModelViewer::getPointProjection(
    const Eigen::Vector3d& pos) const
{
    //Requested 3d point
    GLdouble x = pos.x();
    GLdouble y = pos.y();
    GLdouble z = pos.z();

    //Windows coordinates
    GLdouble wx = 0.0;
    GLdouble wy = 0.0;
    GLdouble wz = 0.0;

    //Retrieve the current active matrices
    GLint viewport[4];
    GLdouble model[16];
    GLdouble projection[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    //Calculate the screen projection
    GLint success = gluProject(
        x, y, z, 
        model, projection, viewport, 
        &wx, &wy, &wz);

    if (success != GL_TRUE || wz < 0.0 || wz > 1.0) {
        return Eigen::Vector2d(-1.0, -1.0);
    } else {
        return Eigen::Vector2d(wx, _height-wy);
    }
}

}

