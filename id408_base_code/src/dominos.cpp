/**
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

/* \mainpage My Personal Index Page
 *
 * \section intro_sec Introduction
 *
 * This is the introduction.
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step 1: Opening the box
 *  
 * etc...
 */

/* 
 * Base code for ID 408 - Technology and Animation
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "id408_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace id408
{
	/**  This is the constructor 
	 * This is the documentation block for the constructor.
	 * \n
	 * b2EdgeShape shape
	 *    It is used repeatedly to create the shapes of multiple objects, ie the dominos, the revolving shelves, the horizontal shelf, and the pulley system
	 * \n
	 * b2Body b1
	 *    It is the pointer to the ground and the two spheres
	 * \n
	 * b2Body* b2
	 *    It is the string to the pendulum.
	 * \n
	 * b2Body* b4
	 *    It is the box of the pendulum that knocks the dominos off
	 * \n
	 * b2Body* ground
	 *    It is the ground on which dominos are presesnt
	 * \n
	 * b2Body* body
	 *      used to define dominos individually
	 * \n
	 * b2Body* body2 
	 *    It  stands for the revolving vertical shelves.
	 *    It has been used multiple times to define the shelves individually.
	 *    
	 */ 
	dominos_t::dominos_t()
	{
		//Ground
		/* \var b1 
		 * \brief pointer to the body ground 
		 */ 
		b2Body* b1;  
		{
			
			b2EdgeShape shape; 
			shape.Set(b2Vec2(-190.0f, 0.0f), b2Vec2(190.0f, 0.0f));
			b2BodyDef bd; 
			b2FixtureDef groundfd;
			groundfd.shape = &shape;
			groundfd.friction = 0.f;
			groundfd.restitution = 0.1f;
			b1 = m_world->CreateBody(&bd); 
			b1->CreateFixture(&groundfd);
		}
		
		{
			b2BodyDef killer_body_def; //! killer_body_def is b2BodyDef type variable for killer's body definition
			killer_body_def.position.Set(-38.f, 10.f);
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

		for(int i=-1800;i<1000;i++)
		{
				//Many Small balls
				b2Body* spherebody1;
				b2CircleShape circle;
				circle.m_radius = 0.09;
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 0.00000000009f;
				ballfd.friction = 0.f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(0.35*i,45+rand()%500);
				spherebody1 = m_world->CreateBody(&ballbd);
				spherebody1->CreateFixture(&ballfd);
		}
			for(int i=-1800;i<1000;i++)
		{
				//Many Small balls
				b2Body* spherebody1;
				b2CircleShape circle;
				circle.m_radius = 0.09;
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 0.00000000009f;
				ballfd.friction = 0.f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(0.35*i,45+rand()%500);
				spherebody1 = m_world->CreateBody(&ballbd);
				spherebody1->CreateFixture(&ballfd);
		}
		for(int i=-1800;i<1000;i++)
		{
				//Many Small balls
				b2Body* spherebody1;
				b2CircleShape circle;
				circle.m_radius = 0.09;
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 0.00000000009f;
				ballfd.friction = 0.f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(0.35*i,45+rand()%500);
				spherebody1 = m_world->CreateBody(&ballbd);
				spherebody1->CreateFixture(&ballfd);
		}

		for(int i=0;i<2;i++)
		{
				//Many Small balls
				b2Body* spherebody1;
				b2CircleShape circle;
				circle.m_radius = 0.09;
				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 0.0725f;
				ballfd.friction = 0.f;
				ballfd.restitution = 0.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(0,45+500*i);
				
				spherebody1 = m_world->CreateBody(&ballbd);
				spherebody1->CreateFixture(&ballfd);
				if(i==1)
				spherebody1->SetGravityScale( 0.1 );
		}

		//The see-saw system at the bottom
		{
			//The triangle wedge
			b2Body* sbody;
			b2PolygonShape poly;
			b2Vec2 vertices[3];
			vertices[0].Set(-1,0);
			vertices[1].Set(1,0);
			vertices[2].Set(0,1.5);
			poly.Set(vertices, 3);
			b2FixtureDef wedgefd;
			wedgefd.shape = &poly;
			wedgefd.density = 10.0f;
			wedgefd.friction = 0.0f;
			wedgefd.restitution = 0.0f;
			b2BodyDef wedgebd;
			wedgebd.position.Set(30.0f, 0.0f);
			sbody = m_world->CreateBody(&wedgebd);
			sbody->CreateFixture(&wedgefd);

			//The plank on top of the wedge
			b2PolygonShape shape;
			shape.SetAsBox(15.0f, 0.2f);
			b2BodyDef bd2;
			bd2.position.Set(30.0f, 1.5f);
			bd2.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd2);
			b2FixtureDef *fd2 = new b2FixtureDef;
			fd2->density = 1.f;
			fd2->shape = new b2PolygonShape;
			fd2->shape = &shape;
			body->CreateFixture(fd2);

			b2RevoluteJointDef jd;
			b2Vec2 anchor;
			anchor.Set(30.0f, 1.5f);
			jd.Initialize(sbody, body, anchor);
			m_world->CreateJoint(&jd);

		/*The light box on the right side of the see-saw
				 b2PolygonShape shape2;
				 shape2.SetAsBox(2.0f, 2.0f);
				 b2BodyDef bd3;
				 bd3.position.Set(40.0f, 2.0f);
				 bd3.type = b2_dynamicBody;
				 b2Body* body3 = m_world->CreateBody(&bd3);
				 b2FixtureDef *fd3 = new b2FixtureDef;
				 fd3->density = 0.01f;
				 fd3->shape = new b2PolygonShape;
				 fd3->shape = &shape2;
				 body3->CreateFixture(fd3);
				 */
		}
	}
	/// testing
	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
