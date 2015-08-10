/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for ID 408 Technology and Animation
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Sumant Rao
 
 */

//! These are user defined include files
//! Included in double quotes - the path to find these has to be given at compile time
#include "id408_base.hpp"
#include "render.hpp"


#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/freeglut.h"
#endif

#include <cstring>
#include <iostream>
using namespace std;

#include "dominos.hpp"
//! The namespace protects the global variables and other names from
//! clashes in scope. Read about the use of named and unnamed
//! namespaces in C++ Figure out where all the datatypes used below
//! are defined
namespace id408
{
    /**  This is the constructor
     * This is the documentation block for the constructor.
     */
    
    dominos_t::dominos_t()
    {
        b2Body* b1; //!b1 is b2Body type variable representing the body of ground\n
        {
            
            b2EdgeShape ground_shape; // this is for shape of ground
            ground_shape.Set(b2Vec2(-50.0f, -8.0f), b2Vec2(65.0f, -8.0f));
            b2BodyDef ground_body_def; // body definition of ground
            b1 = m_world->CreateBody(&ground_body_def);
            b1->CreateFixture(&ground_shape, 0.0f);
        }
        
        {
            
            b2BodyDef *pulley_system_common_body_def = new b2BodyDef;            pulley_system_common_body_def->type = b2_dynamicBody; //! this is the common b2BodyDef variable used for various elements of pulley system.
            
            float y=0.7f, b=2, a=2;
            
			b2PolygonShape pulley_shape; //pulley_shape is pulley shape variable used for two square boxes\n
			pulley_shape.SetAsBox(a, b);
            b2FixtureDef *pulley_fixture_def = new b2FixtureDef; // this is used to define the fixture for pulley shape variable used for two square boxes
            pulley_fixture_def->density = 20.0f;
            
            b2BodyDef open_box_def;//! open_box_def is body definition of open box attached to pulley
            open_box_def.type = b2_dynamicBody;
            open_box_def.position.Set(70.0f, y);
            b2Body* open_box = m_world->CreateBody(&open_box_def);
            b2PolygonShape* box = new b2PolygonShape;
            box->SetAsBox(3,0.2, b2Vec2(0.f,-3.2f), 0);
            b2FixtureDef open_box_fixture;
            open_box_fixture.shape = box;
            open_box_fixture.density = 1.0f;
            open_box_fixture.friction = 0.5f;
            open_box->CreateFixture(&open_box_fixture);
            box->SetAsBox(0.2f, 3.3f, b2Vec2(3.3f, 0), 0);
            open_box_fixture.shape = box;
            open_box->CreateFixture(&open_box_fixture);
            box->SetAsBox(0.2f, 3.3f, b2Vec2(-3.3f, 0), 0);
            open_box_fixture.shape = box;
            open_box->CreateFixture(&open_box_fixture);
            
            b2BodyDef support_def;//support for square box
            b2Body* support = m_world->CreateBody(&support_def);
            b2EdgeShape support_shape;
			support_shape.Set(b2Vec2(79.99f, -1.3f), b2Vec2(80.01f, -1.3f));
            support->CreateFixture(&support_shape, 0.0f);
            
            
			pulley_system_common_body_def->position.Set(80.0f, y);
			b2Body* pulley_body_2 = m_world->CreateBody(pulley_system_common_body_def);
			pulley_body_2->CreateFixture(&pulley_shape, 0.5f); // body of square box 2, 0.4770999
            
			b2PulleyJointDef pulley_joint; //! pulley_joint is b2PulleyJointDef type variable used as definition of joint of pulley
			b2Vec2 anchor1(70.0f, y + b);
			b2Vec2 anchor2(80.0f, y + b);
			b2Vec2 groundAnchor1(70.0f, y + 47);
			b2Vec2 groundAnchor2(80.0f, y + 47);
			pulley_joint.Initialize(open_box, pulley_body_2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.0f);
            
			m_world->CreateJoint(&pulley_joint);
            
        }
        
        
        
        /** BALL ON THE T SHAPE PLATFORM */
        {
            
            b2Body* sphere_body_common; //! sphere_body_common is b2Body shape variable used for all the three spheres
            b2CircleShape circle; // for the two smallest ball
            circle.m_radius = 1.0;
            b2CircleShape circle1; // for the medium sized ball
            circle1.m_radius = 1.5;
            b2CircleShape circle2; // for the largest ball
            circle2.m_radius = 2.0;
            
            
            b2FixtureDef ballfd; // for the two smallest ball
            ballfd.shape = &circle;
            ballfd.density = 8.8f;
            ballfd.friction = 0.1f;
            ballfd.restitution = 0.3f;
            b2FixtureDef ballfd1; // for the medium sized ball circle
            ballfd1.shape = &circle1;
            ballfd1.density = 5.8f;
            ballfd1.friction = 0.1f;
            ballfd1.restitution = 0.1f;
            b2FixtureDef ballfd2; // for the largest ball
            ballfd2.shape = &circle2;
            ballfd2.density = 3.5f;
            ballfd2.friction = 0.1f;
            ballfd2.restitution = 0.f;
            
            b2BodyDef ball_common_body_def; //!ball_common_body_def is b2BodyDef type common body definition for all four balls
            ball_common_body_def.type = b2_dynamicBody;
            
            ball_common_body_def.position.Set(-42.f, 41.4f);
            sphere_body_common = m_world->CreateBody(&ball_common_body_def);
            sphere_body_common->CreateFixture(&ballfd);
            
            ball_common_body_def.position.Set(-37.4f, 41.9f);
            sphere_body_common = m_world->CreateBody(&ball_common_body_def);
            sphere_body_common->CreateFixture(&ballfd1);
            
            ball_common_body_def.position.Set(-33.f, 42.5f);
            sphere_body_common = m_world->CreateBody(&ball_common_body_def);
            sphere_body_common->CreateFixture(&ballfd2);
            
        }
        {
            b2BodyDef killer_body_def; //! killer_body_def is b2BodyDef type variable for killer's body definition
            killer_body_def.position.Set(-58.f, 25.5f);
            killer_body_def.type = b2_dynamicBody;
            b2Body* killer_body = m_world->CreateBody(&killer_body_def); // b2Body* type variable for killer's body
            
            b2PolygonShape killer_mid_body_shape;  // b2PolygonShape type variable for killer's mid_body shape
            killer_mid_body_shape.SetAsBox(2.f, 2.5f, b2Vec2(0,-3.5),0);
            b2FixtureDef *killer_mid_body_fixture = new b2FixtureDef;  // b2FixtureDef * type variable for killer's mid_body fixture
            killer_mid_body_fixture->shape = &killer_mid_body_shape;
            killer_mid_body_fixture->density = 0.000001f;
            
            b2CircleShape killer_face_shape;  // b2CircleShape type variable for killer's face shape
            killer_face_shape.m_radius = 1.0;
            b2FixtureDef *killer_face_fixture = new b2FixtureDef;  // b2FixtureDef * type variable for killer's face fixture
            killer_face_fixture->shape = &killer_face_shape;
            killer_face_fixture->density = 0.000001;
                        
            b2PolygonShape killer_left_leg_shape;  // b2PolygonShape type variable for killer's left leg shape
            killer_left_leg_shape.SetAsBox(0.3f, 2.f, b2Vec2(-1.5,-7),0);
            b2FixtureDef *killer_left_leg_fixture = new b2FixtureDef;  // b2FixtureDef * type variable for killer's left_leg fixture
            killer_left_leg_fixture->shape = &killer_left_leg_shape;
            killer_left_leg_fixture->density = 0.000001f;
            
            b2PolygonShape killer_right_leg_shape;  // b2PolygonShape type variable for killer's right_leg shape
            killer_right_leg_shape.SetAsBox(0.3f, 2.f, b2Vec2(1.5,-7),0);
            b2FixtureDef *killer_right_leg_fixture = new b2FixtureDef;  // b2FixtureDef type variable for killer's right_leg fixture
            killer_right_leg_fixture->shape = &killer_right_leg_shape;
            killer_right_leg_fixture->density = 0.000001f;
            
            b2PolygonShape killer_right_hand_shape; // b2PolygonShape type variable for killer's right_hand shape
            killer_right_hand_shape.SetAsBox(0.3f, 2.f, b2Vec2(2.5,-3),0.3);
            b2FixtureDef *killer_right_hand_fixture = new b2FixtureDef; // b2FixtureDef type variable for killer's right_hand fixture
            killer_right_hand_fixture->shape = &killer_right_hand_shape;
            
            b2PolygonShape killer_left_hand_shape; //b2PolygonShape type variable for killer's left_hand shape
            killer_left_hand_shape.SetAsBox(0.3f, 2.f, b2Vec2(-3.8,-1),-1.6);
            b2FixtureDef *killer_left_hand_fixture = new b2FixtureDef; // type variable for killer's left hand
            killer_left_hand_fixture->shape = &killer_left_hand_shape;
            
            killer_body->CreateFixture(killer_mid_body_fixture);
            killer_body->CreateFixture(killer_face_fixture);
            killer_body->CreateFixture(killer_left_leg_fixture);
            killer_body->CreateFixture(killer_right_leg_fixture);
            killer_body->CreateFixture(killer_right_hand_fixture);
            killer_body->CreateFixture(killer_left_hand_fixture);
        }
        {
            float x=0;
            b2BodyDef human_body_def; //!human_body_def is b2BodyDef type variable for human's body definition
            human_body_def.position.Set(-80.f, 25.5f);
            b2Body* human_body = m_world->CreateBody(&human_body_def); //b2Body* type variable for human's body
            
            b2PolygonShape human_mid_body_shape; // b2PolygonShape type variable for human's face
            human_mid_body_shape.SetAsBox(1.6f, 2.5f, b2Vec2(0-x,-3.5),0);
            b2FixtureDef *human_mid_body_fixture = new b2FixtureDef; // b2FixtureDef type variable for human's mid_body fixture
            human_mid_body_fixture->shape = &human_mid_body_shape;
            
            b2CircleShape human_face_shape; // b2CircleShape type variable for human's face shape
            human_face_shape.m_radius = 1.0;
            b2FixtureDef *human_face_fixture = new b2FixtureDef; // b2FixtureDef type variable for human's face fixture
            human_face_fixture->shape = &human_face_shape;
            
            b2PolygonShape human_left_leg_shape; // b2PolygonShape type variable for human's left leg shape
            human_left_leg_shape.SetAsBox(0.3f, 2.f, b2Vec2(-1.3-x,-7),0);
            b2FixtureDef *human_left_leg_fixture = new b2FixtureDef; // type variable for human's left_leg fixture
            human_left_leg_fixture->shape = &human_left_leg_shape;
            
            b2PolygonShape human_right_leg_shape; // b2PolygonShape type variable for human's right_leg shape
            human_right_leg_shape.SetAsBox(0.3f, 2.f, b2Vec2(1.3-x,-7),0);
            b2FixtureDef *human_right_leg_fixture = new b2FixtureDef; // type variable for human's right_leg fixture
            human_right_leg_fixture->shape = &human_right_leg_shape;
            
            b2PolygonShape human_right_hand_shape; // b2PolygonShape type variable for human's right_hand shape
            human_right_hand_shape.SetAsBox(0.3f, 2.2f, b2Vec2(2-x,1.2),3);
            b2FixtureDef *human_right_hand_fixture = new b2FixtureDef; // type variable for  human's right_hand fixture
            human_right_hand_fixture->shape = &human_right_hand_shape;
            
            b2PolygonShape human_left_hand_shape; // b2PolygonShape type variable for human's left_hand shape
            human_left_hand_shape.SetAsBox(0.3f, 2.2f, b2Vec2(-2-x,1.2),-3);
            b2FixtureDef *human_left_hand_fixture = new b2FixtureDef; // b2PolygonShape type variable for human's left_hand shape
            human_left_hand_fixture->shape = &human_left_hand_shape;
            
            human_body->CreateFixture(human_mid_body_fixture);
            human_body->CreateFixture(human_face_fixture);
            human_body->CreateFixture(human_left_leg_fixture);
            human_body->CreateFixture(human_right_leg_fixture);
            human_body->CreateFixture(human_right_hand_fixture);
            human_body->CreateFixture(human_left_hand_fixture);
        }
        {
            b2Body* hydrogen_sphere_body; //!hydrogen_sphere_body is b2Body* type variable for hydrogen sphere body
            b2CircleShape hydrogen_sphere_shape;  //b2CircleShape type variable for hydrogen sphere shape
            hydrogen_sphere_shape.m_radius = 1.0f;
            
            b2PolygonShape thread_shape;
            thread_shape.SetAsBox(0.1f, 2.0f, b2Vec2(0.0f, -1.0f), 0.0f);
            
            b2FixtureDef hydrogen_sphere_fixture_def; //b2FixtureDef type variable for hydrogen sphere fixture definition
            hydrogen_sphere_fixture_def.shape = &hydrogen_sphere_shape;
            hydrogen_sphere_fixture_def.density = 10.0f;
            hydrogen_sphere_fixture_def.friction = 0.1f;
            hydrogen_sphere_fixture_def.restitution = 0.0f;
            b2BodyDef hydrogen_sphere_body_def; // b2BodyDef type variable for hydrogen sphere body definition
            hydrogen_sphere_body_def.type = b2_dynamicBody;
            hydrogen_sphere_body_def.position.Set(-72.f, -5.f);
            //hydrogen_sphere_body_def.fixedRotation = true;
            hydrogen_sphere_body = m_world->CreateBody(&hydrogen_sphere_body_def);
            hydrogen_sphere_body->CreateFixture(&hydrogen_sphere_fixture_def);
            //hydrogen_sphere_body->CreateFixture(&thread_shape, 0.0f);
            hydrogen_sphere_body->SetGravityScale(-1.0f);
            
            hydrogen_sphere_body_def.position.Set(-54.f, -5.f);
            hydrogen_sphere_body = m_world->CreateBody(&hydrogen_sphere_body_def);
            hydrogen_sphere_body->CreateFixture(&hydrogen_sphere_fixture_def);
            //hydrogen_sphere_body->CreateFixture(&thread_shape, 0.0f);
            hydrogen_sphere_body->SetGravityScale(-1.0f);
            
            hydrogen_sphere_shape.m_radius = 0.5f;
            hydrogen_sphere_fixture_def.density = 5.0f;
            hydrogen_sphere_fixture_def.friction = 0.0f;
            hydrogen_sphere_fixture_def.restitution = 0.6f;
            hydrogen_sphere_body_def.type = b2_dynamicBody;
            hydrogen_sphere_body_def.position.Set(-78.f, 17.f);
            hydrogen_sphere_body = m_world->CreateBody(&hydrogen_sphere_body_def);
            hydrogen_sphere_body->CreateFixture(&hydrogen_sphere_fixture_def);
            hydrogen_sphere_body->SetLinearVelocity(b2Vec2(5,2));
            
        }
        {
            b2BodyDef hinge_body_def; //!hinge_body_def is b2BodyDef type variable for hinge body definition
            hinge_body_def.position.Set(-80.f, 4.f);
            b2Body* hinge_body = m_world->CreateBody(&hinge_body_def);
            
            b2PolygonShape shape1; // b2PolygonShape type variable for body1 = box below human
            shape1.SetAsBox(6.f, 4.f, b2Vec2(0,8),0);
            b2FixtureDef *fd1 = new b2FixtureDef;
            fd1->shape = &shape1;
            
            b2PolygonShape shape2; // b2PolygonShape type variable for body2 = box below killer
            shape2.SetAsBox(8.f, 4.f, b2Vec2(17,8),0);
            b2FixtureDef *fd2 = new b2FixtureDef;
            fd2->shape = &shape2;
            
            b2PolygonShape shape3; // b2PolygonShape type variable for body3 = tower
            shape3.SetAsBox(0.4f, 12.f, b2Vec2(36,24),0);
            b2FixtureDef *fd3 = new b2FixtureDef;
            fd3->shape = &shape3;

            b2PolygonShape shape4; // b2PolygonShape type variable for body4 = box below tower
            shape4.SetAsBox(4.f, 4.f, b2Vec2(34,8),0);
            b2FixtureDef *fd4 = new b2FixtureDef;
            fd4->shape = &shape4;
            
            b2PolygonShape shape10; //! shape10 is b2PolygonShape type variable for body 10. body10 = horizontal bar
            shape10.SetAsBox(36.f, 0.4f, b2Vec2(-20.f,0.f),0);
            
            b2PolygonShape shape11; //!shape11 b2PolygonShape type variable for body11. body11 = vertical bar.
            shape11.SetAsBox(0.2f, 3.0f , b2Vec2(-56.f,1.8f),0);
            
			b2PolygonShape shape12; //!shape11 b2PolygonShape type variable for body11. body11 = vertical bar.
            shape12.SetAsBox(0.2f, 6.0f , b2Vec2(-59.f,1.4f),0);
            b2PolygonShape shape13; //!shape11 b2PolygonShape type variable for body11. body11 = vertical bar.
            shape13.SetAsBox(9.f, 0.3f , b2Vec2(-52.f,-6.5f),-0.2);
            
            b2BodyDef bd10; // b2BodyDef type variable for body10
            bd10.position.Set(-44.0f, 40.0f);
            bd10.type = b2_dynamicBody;
            b2Body* body10 = m_world->CreateBody(&bd10);
            
            b2FixtureDef *fd10 = new b2FixtureDef; // b2FixtureDef type variable for body10
            fd10->density = 0.52f;
            fd10->restitution = 0.0f;
            fd10->shape = new b2PolygonShape;
            fd10->shape = &shape10;
            
            b2FixtureDef *fd11 = new b2FixtureDef; // b2BodyDef type variable for body11
            fd11->density = 0.2f;
            fd11->shape = new b2PolygonShape;
            fd11->shape = &shape11;
            
            b2FixtureDef *fd12 = new b2FixtureDef; // b2FixtureDef type variable for body10
            fd12->density = 0.2f;
            fd12->restitution = 0.0f;
            fd12->shape = new b2PolygonShape;
            fd12->shape = &shape12;
            
            b2FixtureDef *fd13 = new b2FixtureDef; // b2FixtureDef type variable for body10
            fd13->density = 0.2f;
            fd13->restitution = 0.0f;
            fd13->shape = new b2PolygonShape;
            fd13->shape = &shape13;
            
            body10->CreateFixture(fd10);
            body10->CreateFixture(fd11);
            body10->CreateFixture(fd12);
            body10->CreateFixture(fd13);
            
            b2BodyDef bd12; // b2BodyDef type variable for body12 which is hidden object.
            bd12.position.Set(-44.0f, 40.0f);
            b2Body* body12 = m_world->CreateBody(&bd12);
			
            b2RevoluteJointDef jointDef10; //!jointDef10 is b2RevoluteJointDef type variable used for the revolute joint
            jointDef10.bodyA = body10;
            jointDef10.bodyB = body12;
			
            jointDef10.localAnchorA.Set(0.1,0);
            jointDef10.localAnchorB.Set(0.1,0);
            jointDef10.collideConnected = false;
            
            m_world->CreateJoint(&jointDef10);

            hinge_body->CreateFixture(fd1);
            hinge_body->CreateFixture(fd2);
            hinge_body->CreateFixture(fd3);
            hinge_body->CreateFixture(fd4);
            
            {
                b2BodyDef hanger_body_def; //!hinge_body_def is b2BodyDef type variable for hinge body definition
                hanger_body_def.position.Set(-63, 0);
                b2Body* hanger_body = m_world->CreateBody(&hanger_body_def);
                
                float32 a = -3.0f;
                b2Vec2 h(63.0f, a);
                b2BodyDef bodyDef;
                bodyDef.position = -h;
                b2Body* body = m_world->CreateBody(&bodyDef);
                
                b2PolygonShape shape;
                shape.SetAsBox(0.4f, 5.f);
                body->CreateFixture(&shape, 20.f);
                
                b2Vec2 p(63.0f, 0.f);
                b2BodyDef bodyDef1;
                bodyDef1.type = b2_dynamicBody;
                bodyDef1.position = -p;
                b2Body* body1 = m_world->CreateBody(&bodyDef1);
                b2PolygonShape shape1;
                shape1.SetAsBox(9.f, 0.4f);
                body1->CreateFixture(&shape1, 20.f);
                
                b2Vec2 h111(0.0f, 2.5f);
                b2RevoluteJointDef jointDef;
                jointDef.bodyA = hanger_body;
                jointDef.bodyB = body1;
                jointDef.localAnchorA.SetZero();
                jointDef.localAnchorB=h111;
                m_world->CreateJoint(&jointDef);
            }

        }
        
        //left side wall from which the ball will deflect to hit the dominos
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.25f, 11.0f);
            
            b2BodyDef bd;
            bd.position.Set(-38.0f, 6.0f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
			
		}
		
		{
            b2PolygonShape shape;
            shape.SetAsBox(4.0f, 0.2f);
            
            b2BodyDef bd;
            bd.position.Set(-34.0f, -2.0f);
            bd.angle = -0.1f * b2_pi;
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
			
		}
		
        //Horizontal shelf(middle) for dominos
        {
            b2PolygonShape shape;
            shape.SetAsBox(25.0f, 0.25f);
            
            b2BodyDef bd;
            bd.position.Set(-1.0f, 0.0f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
        }
        
        //Horizontal shelf(top) for dominos
        {
            b2PolygonShape shape;
            shape.SetAsBox(25.0f, 0.25f);
            
            b2BodyDef bd;
            bd.position.Set(-1.0f, 8.0f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
        }
        
        //Dominos(top)
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.15f, 3.0f);
            
            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 20.0f;
            fd.friction = 0.1f;
            
            for (int i = 0; i < 50; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-25.5f + 1.0f * i, 11.0f);
                b2Body* body = m_world->CreateBody(&bd);
                body->CreateFixture(&fd);
            }
        }
        
        
        //Dominos(middle)
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.15f, 3.0f);
            
            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 20.0f;
            fd.friction = 0.1f;
            
            for (int i = 0; i < 50; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-25.5f + 1.0f * i, 3.0f);
                b2Body* body = m_world->CreateBody(&bd);
                body->CreateFixture(&fd);
            }
        }
        
        
        
        //Dominos(bottom)
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.15f, 3.0f);
            
            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 20.0f;
            fd.friction = 0.1f;
            
            for (int i = 0; i < 50; ++i)
            {
                b2BodyDef bd;
                bd.type = b2_dynamicBody;
                bd.position.Set(-25.5f + 1.0f * i, -5.0f);
                b2Body* body = m_world->CreateBody(&bd);
                body->CreateFixture(&fd);
            }
        }
        
        //The revolving vertical bar(right)
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.2f, 6.0f);
            
            b2BodyDef bd;
            bd.position.Set(25.0f, -2.0f);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            
            b2BodyDef bd2;
            bd2.position.Set(25.0f, 0.0f);
            b2Body* body2 = m_world->CreateBody(&bd2);
            
            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }
        
        //The revolving vertical bar(left)
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.2f, 6.0f);
            
            b2BodyDef bd;
            bd.position.Set(-27.0f, 6.0f);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.0f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            
            b2BodyDef bd2;
            bd2.position.Set(-27.0f, 8.0f);
            b2Body* body2 = m_world->CreateBody(&bd2);
            
            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }
        
        
        
		//incline on which balls on the T-shaped platform fall		
		{
			b2PolygonShape shape;
			shape.SetAsBox(6.0f,0.2f);
			
			b2BodyDef bd;
			bd.position.Set(-22.5f,33.0f);
			bd.angle=-0.1f*b2_pi;
			
			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&shape,0.0f);
		}
		
		//holes into which balls will fall
		//largest
		{
			b2PolygonShape shape;
            shape.SetAsBox(2.0f, 0.1f);
            
            b2BodyDef bd;
            bd.position.Set(-14.0f, 27.0f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
            
            b2BodyDef bd_left;
            bd_left.position.Set(-16.5f,29.0f);
            bd_left.angle = 0.5 * b2_pi;
            b2Body* ground_left = m_world->CreateBody(&bd_left);
            ground_left->CreateFixture(&shape,0.0f);

            b2BodyDef bd_right;
            bd_right.position.Set(-11.5f,29.0f);
            bd_right.angle = 0.5 * b2_pi;
            b2Body* ground_right = m_world->CreateBody(&bd_right);
            ground_right->CreateFixture(&shape,0.0f);
		}

        {
            b2PolygonShape shape;
            shape.SetAsBox(3.0f, 0.1f);
            
            b2BodyDef bd;
            bd.position.Set(-8.5f, 30.8f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
        }
		
        //second largest
        {
            b2PolygonShape shape;
            shape.SetAsBox(1.5f, 0.1f);
            
            b2BodyDef bd;
            bd.position.Set(-3.5f, 28.0f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
            
            b2BodyDef bd_left;
            bd_left.position.Set(-5.5f,29.4f);
            bd_left.angle = 0.5 * b2_pi;
            b2Body* ground_left = m_world->CreateBody(&bd_left);
            ground_left->CreateFixture(&shape,0.0f);
            
            b2BodyDef bd_right;
            bd_right.position.Set(-2.0f,29.5f);
            bd_right.angle = 0.5 * b2_pi;
            b2Body* ground_right = m_world->CreateBody(&bd_right);
            ground_right->CreateFixture(&shape,0.0f);
        }

        {
            b2PolygonShape shape;
            shape.SetAsBox(2.0f, 0.1f);
            
            b2BodyDef bd;
            bd.position.Set(0.0f, 30.6f);
            b2Body* ground = m_world->CreateBody(&bd);
            ground->CreateFixture(&shape, 0.0f);
        }
		
        //balance into which the last ball falls
        {
            b2PolygonShape shape10; //! shape10 is b2PolygonShape type variable for body 10. body10 = horizontal bar
            shape10.SetAsBox(6.f, 0.4f, b2Vec2(0.f,0.f),0);
            
            b2PolygonShape shape11; //!shape11 b2PolygonShape type variable for body11. body11 = vertical bar.
            shape11.SetAsBox(0.2f, 2.0f , b2Vec2(-5.f,0.f),0);
            
            
            b2BodyDef bd10; // b2BodyDef type variable for body10
            bd10.position.Set(0.0f, 25.0f);
            bd10.type = b2_dynamicBody;
            b2Body* body10 = m_world->CreateBody(&bd10);
            
            b2FixtureDef *fd10 = new b2FixtureDef; // b2FixtureDef type variable for body10
            fd10->density = 0.52f;
            fd10->restitution = 0.0f;
            fd10->shape = new b2PolygonShape;
            fd10->shape = &shape10;
            
            b2FixtureDef *fd11 = new b2FixtureDef; // b2BodyDef type variable for body11
            fd11->density = 0.2f;
            fd11->shape = new b2PolygonShape;
            fd11->shape = &shape11;
            
            
            body10->CreateFixture(fd10);
            body10->CreateFixture(fd11);
            
            b2BodyDef bd12; // b2BodyDef type variable for body12 which is hidden object.
            bd12.position.Set(0.0f, 25.0f);
            b2Body* body12 = m_world->CreateBody(&bd12);
            
            b2RevoluteJointDef jointDef10; //!jointDef10 is b2RevoluteJointDef type variable used for the revolute joint
            jointDef10.bodyA = body10;
            jointDef10.bodyB = body12;
            
            jointDef10.localAnchorA.Set(+0.5,0.1);
            jointDef10.localAnchorB.Set(-0.5,-0.1);
            jointDef10.collideConnected = false;
            
            m_world->CreateJoint(&jointDef10);
            
            
        }
        
        //lower incline from which ball falls on and hits dominos after hitting the wall 
		{
			b2PolygonShape shape;
			shape.SetAsBox(29.0f,0.21f);
			
			b2BodyDef bd;
			bd.position.Set(-4.5f,20.0f);
			bd.angle=-0.97f*b2_pi;
			
			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&shape,0.0f);
		}
		
		//horizontal rotating bars on the right
		//first
		{
            b2PolygonShape shape;
            shape.SetAsBox(6.0f, 0.2f);
            
            b2BodyDef bd;
            bd.position.Set(31.5f, 11.0f);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            
            
            b2BodyDef bd2;
            bd2.position.Set(31.5f, 11.0f);
            b2Body* body2 = m_world->CreateBody(&bd2);
            
            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }
        
        //second
        {
            b2PolygonShape shape;
            shape.SetAsBox(6.0f, 0.2f);
            
            b2BodyDef bd;
            bd.position.Set(41.5f, 13.0f);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            
            
            b2BodyDef bd2;
            bd2.position.Set(41.5f, 13.0f);
            b2Body* body2 = m_world->CreateBody(&bd2);
            
            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }
        
        //third
        {
			b2PolygonShape shape;
            shape.SetAsBox(6.0f, 0.2f);
            
            b2BodyDef bd;
            bd.position.Set(51.5f, 11.0f);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            
            
            b2BodyDef bd2;
            bd2.position.Set(51.5f, 11.0f);
            b2Body* body2 = m_world->CreateBody(&bd2);
            
            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }
 
		//fourth (which has a sphere on it)
        {
			b2PolygonShape shape;
            shape.SetAsBox(6.0f, 0.2f);
            
            b2BodyDef bd;
            bd.position.Set(61.5f, 12.5f);
            bd.type = b2_dynamicBody;
            b2Body* body = m_world->CreateBody(&bd);
            b2FixtureDef *fd = new b2FixtureDef;
            fd->density = 1.f;
            fd->shape = new b2PolygonShape;
            fd->shape = &shape;
            body->CreateFixture(fd);
            
            
            b2BodyDef bd2;
            bd2.position.Set(61.5f, 12.5f);
            b2Body* body2 = m_world->CreateBody(&bd2);
            
            b2RevoluteJointDef jointDef;
            jointDef.bodyA = body;
            jointDef.bodyB = body2;
            jointDef.localAnchorA.Set(0,0);
            jointDef.localAnchorB.Set(0,0);
            jointDef.collideConnected = false;
            m_world->CreateJoint(&jointDef);
        }
        
        //sphere on the platform
        {
			b2Body* sbody;
			b2CircleShape circle;
			circle.m_radius = 2.5f;
			
			b2FixtureDef ballfd;
			ballfd.shape = &circle;
			ballfd.density = 1.0f;
			ballfd.friction = 0.0f;
			ballfd.restitution = 0.0f;
			b2BodyDef ballbd;
			ballbd.type = b2_dynamicBody;
			ballbd.position.Set(61.5f, 15.5f);
			sbody = m_world->CreateBody(&ballbd);
			sbody->CreateFixture(&ballfd);
			
		}
		
		{
			b2BodyDef cross_def, support_def; //!cross_def is body definition for crosses above the pulley
			cross_def.position.Set(81.6f, 52.7f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(81.6f, 52.7f);
			b2Body *cross, *support;
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			b2PolygonShape box1, box2;
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			b2RevoluteJointDef jointDef;
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			cross_def.position.Set(80.6f, 57.2f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(80.6f, 57.2f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			cross_def.position.Set(80.6f, 48.2f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(80.6f, 48.2f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.01f);
			cross->CreateFixture(&box2, 0.01f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			/*cross_def.position.Set(81.6f, 61.7f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(81.6f, 61.7f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);*/
			
			cross_def.position.Set(76.8f, 57.7f);//horizontal crosses
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(76.8f, 57.7f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			cross_def.position.Set(73.3f, 56.7f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(73.3f, 56.7f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			cross_def.position.Set(68.8f, 57.7f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(68.8f, 57.7f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			cross_def.position.Set(64.3f, 56.7f);
			cross_def.type = b2_dynamicBody;
			support_def.position.Set(64.3f, 56.7f);
			cross = m_world->CreateBody(&cross_def);
			support = m_world->CreateBody(&support_def);
			box1.SetAsBox(0.2f, 2.5f);
			box2.SetAsBox(2.5f, 0.2f);
			cross->CreateFixture(&box1, 0.00001f);
			cross->CreateFixture(&box2, 0.00001f);
			
			jointDef.bodyA = cross;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
		}
		
		{
			b2BodyDef motor_def, support_def;//motors
			motor_def.position.Set(-26.0f, 48.7f);
			support_def.position.Set(-26.0f, 48.7f);
			motor_def.type = b2_kinematicBody;
			b2Body* motor = m_world->CreateBody(&motor_def);//! motor is b2Body* type variable used for self rotating motors
			b2Body* support = m_world->CreateBody(&support_def);
			b2PolygonShape motor_shape;
			motor_shape.SetAsBox(0.2f, 3.1f, b2Vec2(0,0), 0);
			motor->CreateFixture(&motor_shape, 0.0f);
			motor_shape.SetAsBox(0.2f, 3.1f, b2Vec2(0,0), 2*b2_pi/3);
			motor->CreateFixture(&motor_shape, 0.0f);
			motor_shape.SetAsBox(0.2f, 3.1f, b2Vec2(0,0), 4*b2_pi/3);
			motor->CreateFixture(&motor_shape, 0.0f);
			motor->SetAngularVelocity( 1.04f );
			
			b2RevoluteJointDef jointDef;
			jointDef.bodyA = motor;
			jointDef.bodyB = support;
			jointDef.localAnchorA.Set(0,0);
			jointDef.localAnchorB.Set(0,0);
			jointDef.collideConnected = false;
			m_world->CreateJoint(&jointDef);
			
			for(int i=0; i<7; i++){
				motor_def.position.Set(-24.0f-(2.5f*i), 54.7f+i);
				support_def.position.Set(-24.0f-(2.5f*i), 54.7f+i);
				motor = m_world->CreateBody(&motor_def);
				support = m_world->CreateBody(&support_def);
				motor_shape.SetAsBox(0.2f, 3.1f, b2Vec2(0,0), 0);
				motor->CreateFixture(&motor_shape, 0.0f);
				motor_shape.SetAsBox(0.2f, 3.1f, b2Vec2(0,0), 2*b2_pi/3);
				motor->CreateFixture(&motor_shape, 0.0f);
				motor_shape.SetAsBox(0.2f, 3.1f, b2Vec2(0,0), 4*b2_pi/3);
				motor->CreateFixture(&motor_shape, 0.0f);
				motor->SetAngularVelocity( 1.04f );
			
				jointDef.bodyA = motor;
				jointDef.bodyB = support;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				m_world->CreateJoint(&jointDef);
			}
			
		}
			
		{
			b2BodyDef top_def;//topmost platform with balls
			top_def.position.Set(19.7f, 56.1f);
			b2Body* top = m_world->CreateBody(&top_def);
			b2PolygonShape top_shape;
			top_shape.SetAsBox(42.0f, 0.2f);
			top->CreateFixture(&top_shape, 0.0f);
			
			b2PolygonShape base[10];
			for(int i=0; i<10; i++){
				base[i].SetAsBox(1.0f, 0.3f, b2Vec2(40.9f-i*8, 0.3f), 0.0f);
				top->CreateFixture(&base[i], 0.0f);
			}
				
		}
		
		{
			b2BodyDef ball_def;// balls on top platform
			ball_def.type = b2_dynamicBody;
			b2CircleShape ball_shape;
			ball_shape.m_radius = 1.2f;
			b2Body* ball;//! ball is b2Body* type variable which represents all balls on top platform
			b2FixtureDef ballf;
			ballf.shape = &ball_shape;
			ballf.restitution = 1.0f;
			ballf.density = 0.00001f;
			for(int i=0; i<9; i++){
				ball_def.position.Set(60.9f-i*8, 56.7f);
				ball = m_world->CreateBody(&ball_def);
				ball->CreateFixture(&ballf);
			}
			ballf.restitution = 0.0f;
			ball_def.position.Set(60.9f-72.0f, 56.7f);
			ball = m_world->CreateBody(&ball_def);
			ball->CreateFixture(&ballf);
		}
		
		{
			b2BodyDef top_def;//support for movable platform
			top_def.position.Set(-89.5f, 50.0f);
			b2Body* top = m_world->CreateBody(&top_def);
			b2PolygonShape box_shape;
			box_shape.SetAsBox(0.01f, 0.01f);
			top->CreateFixture(&box_shape, 0.0f);
			
			b2BodyDef incline_def; //last incline
			incline_def.position.Set(-92.0f+22.9f, 60.0f);
			incline_def.angle = b2_pi/24.5f;
			b2Body* incline = m_world->CreateBody(&incline_def);
			box_shape.SetAsBox(28.0f, 0.2f);
			incline->CreateFixture(&box_shape, 0.0f);
			
		}
		
		{
			b2BodyDef plat_def;//last platform
			plat_def.position.Set(-80.0f, 35.0f);
			b2Body* plat = m_world->CreateBody(&plat_def);
			b2PolygonShape box_shape;
			box_shape.SetAsBox(3.0f, 0.2f);
			plat->CreateFixture(&box_shape, 0.0f);
		}
			
			
	}
    
    
    
    
    sim_t *sim = new sim_t("Dominos", dominos_t::create);
    
}
