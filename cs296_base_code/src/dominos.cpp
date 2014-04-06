	#ifndef _DOMINOS_HPP_
	#define _DOMINOS_HPP_
	#include "dominos.hpp"
	#include "callbacks.hpp"
	#include "cs296_base.hpp"
	#include <iostream>
	#include <unistd.h>
	//#include <dos.h>
	#include <stdio.h>
	//#include <conio.h>
	
	  //! This is the class that sets up the Box2D simulation world
	  //! Notice the public inheritance - why do we inherit the base_sim_t class?
	  
	 
	  
	  class dominos_t : public  cs296::base_sim_t
	  {
	  public:
	  int state;
		int vel;
		bool dummy_f;
		bool dummy1;
	    float x1;
	        b2Body* m_bodyA;//Cart Box
			b2Body* m_bodyB;//Forward Tyre 
			b2Body* m_bodyC;//Back tyre
			b2Body* m_bodyD;//Seat of man
			b2Body* m_bodyE;//Man's Body
			b2Body* m_bodyE1;
			b2Body* m_bodyF;//Man's neck
			b2Body* m_bodyG;//Man's arm
			b2Body* m_bodyH;//Rod connected to the horse
			b2Body* m_bodyI;//rod2
			b2Body* m_bodyJ;//the whip
			b2Body* m_bodyG1;//mass of the horse
			b2Body* m_bodyG2;//thigh1
			b2Body* m_bodyG3;//calf1
			b2Body* m_bodyG4;//thigh2
			b2Body* m_bodyG5;//calf2
			b2Body* m_bodyGA2;//thigh1
			b2Body* m_bodyGA3;//calf1
			b2Body* m_bodyGA4;//thigh2
			b2Body* m_bodyGA5;
			b2Body* m_bodyK;
			b2Body* m_bodyG6;
			b2Body* weapon;
			b2Body *tri;
			b2RevoluteJoint* m_joint;//Cart and forward tyre
			b2RevoluteJoint* m_joint3;//Cart and back tyre
			b2RevoluteJoint* m_joint2;//Man and seat
			b2RevoluteJoint* m_joint4;//Man and head
			b2WeldJoint* m_joint5;//Man and arm
			b2WeldJoint* m_joint6;//Front tyre and Rod
			b2WeldJoint* m_joint7;
			b2RevoluteJoint* m_jointd;
			b2RevoluteJoint* m_joint71;
			b2RevoluteJoint* m_joint72;
			b2RevoluteJoint* m_joint73;
			b2RevoluteJoint* m_joint74;
			b2RevoluteJoint* m_joint75;
			b2RevoluteJoint* m_joint7A2;
			b2RevoluteJoint* m_joint7A3;
			b2RevoluteJoint* m_joint7A4;
			b2RevoluteJoint* m_joint7A5;
			b2RevoluteJoint* m_joint8;
			b2RevoluteJoint* m_joint9;
			b2RevoluteJoint* m_joint10;
			b2RevoluteJoint* m_joint41;
			b2RevoluteJoint* m_jointd1;
			b2RevoluteJoint* m_jointG6;
			b2WeldJoint* m_jointnew;
			b2WeldJoint* m_joinseat;
			b2WeldJoint* joint3;
			b2RevoluteJoint* revoluteJoint82;
			b2RevoluteJointDef revoluteJointDefG6;
			b2RevoluteJointDef revoluteJointDef82; 
			b2RevoluteJointDef revoluteJointDef10;
			b2RevoluteJointDef revoluteJointDef9;
			b2RevoluteJointDef revoluteJointDef8;
			b2RevoluteJointDef revoluteJointDef75;
			b2RevoluteJointDef revoluteJointDef74;
			b2RevoluteJointDef revoluteJointDef73;
			b2RevoluteJointDef revoluteJointDef72;
			b2RevoluteJointDef revoluteJointDef7A5;
			b2RevoluteJointDef revoluteJointDef7A4;
			b2RevoluteJointDef revoluteJointDef7A3;
			b2RevoluteJointDef revoluteJointDef7A2;
			b2RevoluteJointDef revoluteJointDef71;
			b2WeldJointDef revoluteJointDef7;
			b2WeldJointDef revoluteJointDef6;
			b2WeldJointDef revoluteJointDef5;
			b2RevoluteJointDef revoluteJointDef4;
			b2RevoluteJointDef revoluteJointDef3;
			b2RevoluteJointDef revoluteJointDef2;
			b2RevoluteJointDef revoluteJointDef;
			b2RevoluteJointDef revoluteJointDef31;
			b2RevoluteJointDef revoluteJointDef81;
			b2WeldJointDef	weldJointDef;
			b2WeldJointDef jointDef;
			b2Body *mobject;
			b2BodyDef mobjectDef;	
			b2FixtureDef mobjectFix;
		
		b2Body* m_bodyGA6;
	    dominos_t(){
			x1=0;
			state=-1;
			dummy_f=false;//! variables used in step function
			dummy1=false;
	  b2BodyDef bodyDef;//! Body definition for man cart etc.
	  bodyDef.type = b2_dynamicBody;//! Declaring it to be dynamic
	  b2FixtureDef fixtureDef;//! fixture density for objects
	  fixtureDef.density = 100;
	  fixtureDef.filter.groupIndex=-1;//! To make legs collision resistant with each other
	  
	  b2BodyDef bodyDef2; //! Another body definition
	  bodyDef2.type = b2_dynamicBody;
	  b2FixtureDef fixtureDef2;//! another fixture definition
	  fixtureDef2.density = 5;
	  	  
	  b2BodyDef bodyDef1; //! Another definition
	  bodyDef1.type = b2_dynamicBody;
	  b2FixtureDef fixtureDef1;//! another fixture density
	  fixtureDef1.density = 10;
	  b2PolygonShape polygonshape; 
	    
	  b2FixtureDef fix1;
	  fix1.density=5;
	 
	 //! Body definition for ground
	  b2PolygonShape polygonShape;
	  polygonShape.SetAsBox(1, 1); //! a 2x2 rectangle
	  b2BodyDef myBodyDef;
	  myBodyDef.type = b2_dynamicBody;
	  myBodyDef.type = b2_staticBody;
	  myBodyDef.position.Set(0, 0);
	  b2Body* staticBody = m_world->CreateBody(&myBodyDef);
	    
	  //! fixture definition for ground
	  b2FixtureDef myFixtureDef;
	  myFixtureDef.shape = &polygonShape;
	  myFixtureDef.density = 1;
	  myFixtureDef.friction=0.95;
	  polygonShape.SetAsBox( 200, 13, b2Vec2(0, 0), 0);//! ground
	  staticBody->CreateFixture(&myFixtureDef);
	    
	  
		
	 
	 //! defining shapes
	  b2PolygonShape boxShape;//! The box representing cart
	  boxShape.SetAsBox(10,4);//! Dimensions of 20x8
	  
	  b2PolygonShape seat;//! The seat of man in cart
	  seat.SetAsBox(4,1.5);//! Dimensions of 8x3
	  
	  b2PolygonShape man;//! The body of man
	  man.SetAsBox(3,4);//! Dimensions of 6x8
	  
	  b2PolygonShape head;//! Neck of man
	  head.SetAsBox(0.7,1);
	  
	  b2PolygonShape head1;//! Head of horse
	  	b2Vec2 vertices3[5];
		vertices3[0].Set(0,0);
		vertices3[1].Set(0,2.25);
		vertices3[2].Set(3,2.25);
		vertices3[3].Set(1.5,0);
		vertices3[4].Set(5,-1);
     	head1.Set(vertices3,5);
	  
	  b2PolygonShape hand;//! hand of man till elbow
	  hand.SetAsBox(2,1);
 
      b2PolygonShape spear;//! Long spear in hand of man(warrior) on cart
      spear.SetAsBox(6,1);
	
	  b2PolygonShape hand6;//!The arm of man from elbow to palm
	  hand6.SetAsBox(2.5,0.8);
	   
	  b2PolygonShape Rod;//! Rod joining horse to cart
	  Rod.SetAsBox(8.3,0.1);
	  
	  
	  b2PolygonShape stomach;//! The body of horse
	    b2Vec2 vertices1[6];
		vertices1[0].Set(-8,-2.5);
		vertices1[1].Set(-10,1.5);
		vertices1[2].Set(-8,2.5);
		vertices1[3].Set(6,3.5);
		vertices1[4].Set(8,2.5);
		vertices1[5].Set(7,-2.3);
		stomach.Set(vertices1,7);
	  
	  b2PolygonShape thigh1;//! calfs of horse
	  thigh1.SetAsBox(0.4,2.3);
	  
	  b2PolygonShape thigh2;//! Thighs of horse
	  thigh2.SetAsBox(0.6,2.4);
	  
	  b2PolygonShape irene;//! whip in hand of man
	  irene.SetAsBox(6-0.5,0.1);
	  
	  //! Wheels
	  b2CircleShape circleShape1;
	  b2CircleShape circleShape2;
	  circleShape1.m_radius = 3; 
	  circleShape2.m_radius = 3;     
	  
	  //! using m_bodyA for man
	  bodyDef.position.Set(4.5-15, 20);
	  fixtureDef.shape = &boxShape;
	  m_bodyA = m_world->CreateBody(&bodyDef);
	  m_bodyA->CreateFixture( &fixtureDef );
	  m_bodyA->SetAngularDamping(100);
	  
	  //! using m_bodyD for seat man
	  bodyDef.position.Set(14.5-15,18+1);
	  fixtureDef.shape=&seat;
	  m_bodyD=m_world->CreateBody(&bodyDef);
	  m_bodyD->CreateFixture(&fixtureDef);
	  
	  //! using m_bodyE for man
	  bodyDef1.position.Set(18-15,22);
	  fixtureDef1.shape=&man;
	  m_bodyE=m_world->CreateBody(&bodyDef1);
	  m_bodyE->CreateFixture(&fixtureDef1);
	  
	  //! using m_bodyF for head of man
	  bodyDef1.position.Set(18-15,30);
	  fixtureDef1.shape=&head;
	  m_bodyF=m_world->CreateBody(&bodyDef1);
	  m_bodyF->CreateFixture(&fixtureDef1);
	  
	  //! using m_bodyG for man
	  bodyDef1.position.Set(26-15,23);
	  fixtureDef1.shape=&hand;
	  m_bodyG=m_world->CreateBody(&bodyDef1);
	  m_bodyG->CreateFixture(&fixtureDef1);
	  m_bodyG->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);
	  
	  //! using m_bodyG6 for arm of man from elbow to palm
	  bodyDef1.position.Set(21+0.8-15,22.9);
	  fixtureDef1.shape=&hand6;
	  m_bodyG6=m_world->CreateBody(&bodyDef1);
	  m_bodyG6->CreateFixture(&fixtureDef1);
	  m_bodyG6->SetTransform(m_bodyG6->GetPosition(),5*DEGTORAD);
	  
	  //! using m_bodyJ for whip
	  bodyDef1.position.Set(31-15,23);
	  fixtureDef1.shape=&irene;
	  m_bodyJ=m_world->CreateBody( &bodyDef1 );
	  m_bodyJ->CreateFixture( &fixtureDef1 );
	  m_bodyJ->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);
	 
      //! using m_bodyB for front wheel
	  bodyDef.position.Set( 3-15, 10);
	  fixtureDef.shape = &circleShape1;
	  m_bodyB = m_world->CreateBody( &bodyDef );
	  m_bodyB->CreateFixture( &fixtureDef );
	  m_bodyA->SetAngularDamping(10);
	  
	  //!using m_bodyC for rear wheel 
	  bodyDef.position.Set( 6-15, 10);
	  fixtureDef.shape = &circleShape2;
	  m_bodyC = m_world->CreateBody( &bodyDef );
	  m_bodyC->CreateFixture( &fixtureDef );
	  m_bodyA->SetAngularDamping(10);
	  
	  //! using m_bodyH for rod connecting horse to cart
	  bodyDef.position.Set(0,10);
	  fixtureDef.shape= &Rod;
	  m_bodyH = m_world->CreateBody( &bodyDef );
	  m_bodyH->CreateFixture( &fixtureDef );
	  
	  //!using m_bodyG1 for horse body
	  bodyDef.position.Set( 26+4-15, 16);
	  fixtureDef.shape= &stomach;
	  m_bodyG1 = m_world->CreateBody( &bodyDef );
	  m_bodyG1->CreateFixture( &fixtureDef );
	  
	  //! Tiny boxes used to define tail of horse
	  b2PolygonShape shape;
	  shape.SetAsBox(0.8f, 0.225f);
	
	  //! fixture of tail of horse		
	  b2FixtureDef fd;
	  fd.shape = &shape;
	  fd.density = 20.0f;
	  fd.friction = 0.2f;
	  
	  const float32 y = 15.0f;
	  b2Body* prevBody = m_bodyG1;
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(0.5f , y);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	  b2RevoluteJointDef jd;
	  jd.bodyA = m_bodyG1;
	  jd.bodyB = body;
	  jd.collideConnected = false;
	  jd.localAnchorA.Set(-8,0);//the top right corner of the box
	  jd.localAnchorB.Set(0,0);
	  b2RevoluteJoint* fic;
	  fic = (b2RevoluteJoint*)m_world->CreateJoint( &jd);
	  prevBody=body;
	  
	  //!Using several small boxes for tail of horse
	  for (int32 i = 1; i < 6; ++i)
	  {
		 b2BodyDef bd;
	 	 bd.type = b2_dynamicBody;
	     bd.position.Set(0.5f + i, y);
		 b2Body* body = m_world->CreateBody(&bd);
		 body->CreateFixture(&fd);
		 b2Vec2 anchor(float32(i), y);
		 jd.Initialize(prevBody, body,anchor);
		 m_world->CreateJoint(&jd);
		 prevBody = body;
	  }
	  
	  //! using m_bodyG2 for back thigh of horse facing us
	  bodyDef.position.Set( 24.5-15, 14.75);
	  fixtureDef.shape= &thigh2;
	  m_bodyG2 = m_world->CreateBody( &bodyDef );
	  m_bodyG2->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyGA2 for back thigh of horse facing screen
	  bodyDef.position.Set( 24.5-15+1, 14.75);
	  fixtureDef.shape= &thigh2;
	  m_bodyGA2 = m_world->CreateBody( &bodyDef );
	  m_bodyGA2->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyG3 for front thigh of horse facing us
	  bodyDef.position.Set( 27.5+8-15, 14.75);
	  fixtureDef.shape= &thigh2;
	  m_bodyG3 = m_world->CreateBody( &bodyDef );
	  m_bodyG3->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyGA3 for front thigh of horse facing screen
	  bodyDef.position.Set( 27.5+8-15-1, 14.75);
	  fixtureDef.shape= &thigh2;
	  m_bodyGA3 = m_world->CreateBody( &bodyDef );
	  m_bodyGA3->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyG4 for back calf  of horse facing us
	  bodyDef.position.Set( 24.5-15, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyG4 = m_world->CreateBody( &bodyDef);
	  m_bodyG4->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyGA4 for back calf of horse facing screen
	  bodyDef.position.Set( 24.5-15+1, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyGA4 = m_world->CreateBody( &bodyDef);
	  m_bodyGA4->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyG5 for front calf of horse facing us
	  bodyDef.position.Set( 27.5+8-15, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyG5 = m_world->CreateBody( &bodyDef );
	  m_bodyG5->CreateFixture( &fixtureDef );
	  
	  //! using m_bodyGA5 for for front calf of horse
	  bodyDef.position.Set( 27.5+8-15-1, 10.25);
	  fixtureDef.shape= &thigh1;
	  m_bodyGA5 = m_world->CreateBody( &bodyDef );
	  m_bodyGA5->CreateFixture( &fixtureDef );
	  
	  //! using  for head of horse
	  bodyDef.position.Set( 30-15+8, 19);
	  fixtureDef.shape= &head1;
	  m_bodyK = m_world->CreateBody( &bodyDef );
	  m_bodyK->CreateFixture( &fixtureDef );
	  m_bodyK->SetTransform(m_bodyK->GetPosition(),60*DEGTORAD);
	  m_bodyK->SetAngularDamping(10);
	
	
	  	//!JOINTS
	  //! joint connecting cart and front wheel
	  revoluteJointDef.bodyA = m_bodyA;
	  revoluteJointDef.bodyB = m_bodyB;
	  revoluteJointDef.collideConnected = false;
	  revoluteJointDef.localAnchorA.Set(10,-4);
	  revoluteJointDef.localAnchorB.Set(0,0);
	  m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
	 
	  //! joint connecting man to his seat  
      jointDef.bodyA = m_bodyD;
      jointDef.bodyB = m_bodyE;
      jointDef.collideConnected = false;
      jointDef.localAnchorA.Set(0,5.5);
	  jointDef.localAnchorB.Set(0,0);   
      m_jointnew=(b2WeldJoint*)m_world->CreateJoint( &jointDef );
          
      //! joint connecting cart to rear wheel   
	  revoluteJointDef2.bodyA = m_bodyA;
	  revoluteJointDef2.bodyB = m_bodyC;
	  revoluteJointDef2.collideConnected = false;
	  revoluteJointDef2.localAnchorA.Set(-10,-4);
	  revoluteJointDef2.localAnchorB.Set(0,0); 
	  m_joint3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef2 );
	  
	  //! joint connecting seat to cart	 
	  weldJointDef.bodyA = m_bodyA;
	  weldJointDef.bodyB = m_bodyD;
	  weldJointDef.collideConnected = false;
	  weldJointDef.localAnchorA.Set(10,2);
	  weldJointDef.localAnchorB.Set(-4,0);
	  m_joinseat = (b2WeldJoint*)m_world->CreateJoint( &weldJointDef ); 
	   
	  //!Joint connecting man to his neck 
	  revoluteJointDef4.bodyA = m_bodyE;
	  revoluteJointDef4.bodyB = m_bodyF;
	  revoluteJointDef4.collideConnected = false;
	  revoluteJointDef4.localAnchorA.Set(0,5);
	  revoluteJointDef4.localAnchorB.Set(0,0);
	  revoluteJointDef4.enableLimit = false;
	  m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef4);
	   
	  //!joint connecting man to his arm
	  revoluteJointDef5.bodyA = m_bodyE;
	  revoluteJointDef5.bodyB = m_bodyG;
	  revoluteJointDef5.collideConnected = true;
	  revoluteJointDef5.localAnchorA.Set(4,0);
	  revoluteJointDef5.localAnchorB.Set(0,0);
	  m_joint5 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef5);
	  
	  //! joint connecting seat and rod 
	  revoluteJointDef6.bodyA = m_bodyD;
	  revoluteJointDef6.bodyB = m_bodyH;
	  revoluteJointDef6.collideConnected = false;
	  revoluteJointDef6.localAnchorA.Set(0,0);
	  revoluteJointDef6.localAnchorB.Set(-8.3,0.0);
	  m_joint6 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef6);
	   
	  //! joint connecting rod to horse body
	  revoluteJointDef7.bodyA = m_bodyG1;
	  revoluteJointDef7.bodyB = m_bodyH;
	  revoluteJointDef7.collideConnected = false;
	  revoluteJointDef7.localAnchorA.Set(0,0);
	  revoluteJointDef7.localAnchorB.Set(8.3,0.0);
	  m_joint7 = (b2WeldJoint*)m_world->CreateJoint( &revoluteJointDef7);
	   
	  //! joints connecting body to thigh 
	  revoluteJointDef72.bodyA = m_bodyG1;
	  revoluteJointDef72.bodyB = m_bodyG2;
	  revoluteJointDef72.collideConnected = false;
	  revoluteJointDef72.localAnchorA.Set(-6,0);
	  revoluteJointDef72.localAnchorB.Set(0,2.25);
	  revoluteJointDef72.enableLimit = true;
	  revoluteJointDef72.lowerAngle = 5* DEGTORAD;
	  revoluteJointDef72.upperAngle = -5* DEGTORAD;
	  m_joint72 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef72);
	  
      //!joint between front right thigh of horse to body 
	  revoluteJointDef73.bodyA = m_bodyG1;
	  revoluteJointDef73.bodyB = m_bodyG3;
	  revoluteJointDef73.collideConnected = false;
	  revoluteJointDef73.localAnchorA.Set(6,0);
	  revoluteJointDef73.localAnchorB.Set(0,2.25);
	  revoluteJointDef73.enableLimit = true;
	  revoluteJointDef73.enableLimit = true;
	  revoluteJointDef73.lowerAngle = 5* DEGTORAD;
	  revoluteJointDef73.upperAngle = -5* DEGTORAD;
	  m_joint73 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef73);
	   
	  //!joint between the right back thigh and calf 
	  revoluteJointDef74.bodyA = m_bodyG2;
	  revoluteJointDef74.bodyB = m_bodyG4;
	  revoluteJointDef74.collideConnected = false;
	  revoluteJointDef74.localAnchorA.Set(0,-2.25);
	  revoluteJointDef74.localAnchorB.Set(0,2.25);
	  revoluteJointDef74.enableLimit = true;
	  revoluteJointDef74.enableLimit = true;;
	  revoluteJointDef74.lowerAngle = 5* DEGTORAD;
	  revoluteJointDef74.upperAngle = -5* DEGTORAD;
	  m_joint74 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef74);
	   
	  //!joint between right front thigh and calf
	  revoluteJointDef75.bodyA = m_bodyG3;
	  revoluteJointDef75.bodyB = m_bodyG5;
	  revoluteJointDef75.collideConnected = false;
	  revoluteJointDef75.localAnchorA.Set(0,-2.25);
	  revoluteJointDef75.localAnchorB.Set(0,2.25);
	  revoluteJointDef75.enableLimit = true;
	  revoluteJointDef75.enableLimit = true;
	  revoluteJointDef75.lowerAngle = 5* DEGTORAD;
	  revoluteJointDef75.upperAngle = -10* DEGTORAD;
	  m_joint75 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef75);
	   
	  //! joint between left back thigh and body
	  revoluteJointDef7A2.bodyA = m_bodyG1;
	  revoluteJointDef7A2.bodyB = m_bodyGA2;
	  revoluteJointDef7A2.collideConnected = false;
	  revoluteJointDef7A2.localAnchorA.Set(-4.5,0);
	  revoluteJointDef7A2.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A2.enableLimit = true;
	  revoluteJointDef7A2.enableLimit = true;
	  revoluteJointDef7A2.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A2.upperAngle = -10* DEGTORAD;
	  m_joint7A2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A2);
	   
	  //!joint between left front thigh and body
	  revoluteJointDef7A3.bodyA = m_bodyG1;
	  revoluteJointDef7A3.bodyB = m_bodyGA3;
	  revoluteJointDef7A3.collideConnected = false;
	  revoluteJointDef7A3.localAnchorA.Set(4.5,0);
	  revoluteJointDef7A3.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A3.enableLimit = true;
	  revoluteJointDef7A3.enableLimit = true;
	  revoluteJointDef7A3.motorSpeed = 1.0f;
	  revoluteJointDef7A3.maxMotorTorque = 1;
	  revoluteJointDef7A3.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A3.upperAngle = -10* DEGTORAD;
	  m_joint7A3 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A3);
	   
	  //! joint between left back thigh and calf
	  revoluteJointDef7A4.bodyA = m_bodyGA2;
	  revoluteJointDef7A4.bodyB = m_bodyGA4;
	  revoluteJointDef7A4.collideConnected = false;
	  revoluteJointDef7A4.localAnchorA.Set(0,-2.25);
	  revoluteJointDef7A4.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A4.enableLimit = true;
	  revoluteJointDef7A4.enableLimit = true;
	  revoluteJointDef7A4.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A4.upperAngle = -10* DEGTORAD;
	  m_joint7A4 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A4);
	   
	  //!joint between left front thigh and calf
	  revoluteJointDef7A5.bodyA = m_bodyGA3;
	  revoluteJointDef7A5.bodyB = m_bodyGA5;
	  revoluteJointDef7A5.collideConnected = false;
	  revoluteJointDef7A5.localAnchorA.Set(0,-2.25);
	  revoluteJointDef7A5.localAnchorB.Set(0,2.25);
	  revoluteJointDef7A5.enableLimit = true;
	  revoluteJointDef7A5.enableLimit = true;
	  revoluteJointDef7A5.motorSpeed = 1.0f;
	  revoluteJointDef7A5.maxMotorTorque = 1;
	  revoluteJointDef7A5.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef7A5.upperAngle = -10* DEGTORAD;
	  m_joint7A5 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef7A5);  
	  revoluteJointDef8.bodyA = m_bodyG6;
	  revoluteJointDef8.bodyB = m_bodyJ;
	  revoluteJointDef8.collideConnected = false;
	  revoluteJointDef8.localAnchorA.Set(2,0);
	  revoluteJointDef8.localAnchorB.Set(-6+0.5,0);
	  revoluteJointDef8.enableLimit = true;
	  m_joint8 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef8);
	   
	  //!elbow joint of charioter  
	  revoluteJointDefG6.bodyA = m_bodyG6;
	  revoluteJointDefG6.bodyB = m_bodyG;
	  revoluteJointDefG6.collideConnected = false;
	  revoluteJointDefG6.localAnchorA.Set(-2.4,0.1);
	  revoluteJointDefG6.localAnchorB.Set(2.3,-0.1);
	  revoluteJointDefG6.enableLimit = true;
	  m_jointG6 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDefG6);
	  
	  //!joint between rod and horse body 
	  revoluteJointDef9.bodyA = m_bodyG1;
	  revoluteJointDef9.bodyB = m_bodyK;
	  revoluteJointDef9.collideConnected = true;
	  revoluteJointDef9.localAnchorA.Set(7,3);
	  revoluteJointDef9.localAnchorB.Set(0,1);
	  revoluteJointDef9.enableLimit = true;
	  revoluteJointDef9.enableMotor = true;
	  revoluteJointDef9.lowerAngle = 5* DEGTORAD;
	  revoluteJointDef9.upperAngle = -5* DEGTORAD;
	  m_joint9 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef9);
	  
	  state=0;
	  vel=0;
	   
	   //! rear man's body 
	   b2Body *man1;
	   b2BodyDef manDef1;
	   b2FixtureDef manFix1;
	   b2PolygonShape manPoly1;;
	   manPoly1.SetAsBox(3,4);
	   manFix1.friction=0;
	   manFix1.restitution=0;
	   manFix1.shape = &manPoly1;
	   manDef1.position.Set(-12, 29);
	   manDef1.type = b2_dynamicBody;
	   man1 = m_world->CreateBody(&manDef1);
	   man1->CreateFixture(&manFix1);
	  
	  //! rear man's neck
	   b2Body *manh;
	   b2BodyDef manhDef2;
	   b2FixtureDef manhFix2;
		
	   b2PolygonShape manhPoly2;;
	   manhPoly2.SetAsBox(0.7,1);
	   manhFix2.shape = &manhPoly2;
	   manhFix2.restitution = 0;
	   manhDef2.position.Set(-12, 34);
	   manhDef2.type = b2_dynamicBody;
	   manh = m_world->CreateBody(&manhDef2);
	   manh->CreateFixture(&manhFix2);
		
	   //! joint between rear man's body and his head
       b2RevoluteJoint* revoluteJoint42;
       b2RevoluteJointDef revoluteJointDef42;
 	   revoluteJointDef42.bodyA = man1;
	   revoluteJointDef42.bodyB = manh;
	   revoluteJointDef42.collideConnected = false;
	   revoluteJointDef42.localAnchorA.Set(0,4);//the top right corner of the box
	   revoluteJointDef42.localAnchorB.Set(0,-1);//center of the circle
	   revoluteJointDef42.enableLimit = false;  
	   revoluteJointDef42.enableMotor = false; 
	   revoluteJointDef42.motorSpeed = 0.4f;
	   revoluteJointDef42.maxMotorTorque=10000000;
	   revoluteJoint42 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef42);
        
        //! joint between rear man's body and cart
       b2WeldJoint* joint22;
       b2WeldJointDef jointDef22;
       jointDef22.bodyA = man1;
       jointDef22.bodyB = m_bodyA;
       jointDef22.collideConnected = true;
       jointDef22.localAnchorA.Set(0,-4);
	   jointDef22.localAnchorB.Set(0,4);
       joint22=(b2WeldJoint*)m_world->CreateJoint( &jointDef22 );   
      
      //! rear man's hand from elbow to palm
       b2Body* manhand;
	   b2BodyDef manhandDef;
	   b2FixtureDef manhandFix;     
       b2PolygonShape manhandPoly;;
	   manhandPoly.SetAsBox(2,1);
	   manhandFix.shape=&manhandPoly;
	   manhandDef.type = b2_dynamicBody;
	   manhandDef.position.Set(-9,28);
	   manhand=m_world->CreateBody(&manhandDef);
	   manhand->CreateFixture(&manhandFix);
	   manhand->SetTransform(manhand->GetPosition(),30*DEGTORAD);
	  
	   //!rear man's hand till elbow
       b2BodyDef m_bodyGA6Def;
       b2FixtureDef m_bodyGA6Fix;     
       b2PolygonShape m_bodyGA6Poly;;
       m_bodyGA6Poly.SetAsBox(2.5,0.8);
       m_bodyGA6Fix.shape=&m_bodyGA6Poly;
       m_bodyGA6Def.type = b2_dynamicBody;
       m_bodyGA6Def.position.Set(-30,28);
       m_bodyGA6=m_world->CreateBody(&m_bodyGA6Def);
       m_bodyGA6->CreateFixture(&m_bodyGA6Fix);
       m_bodyGA6->SetTransform(m_bodyGA6->GetPosition(),110*DEGTORAD);
	   
	  //! joint between rear man and his hand
	  b2RevoluteJoint* revoluteJoint52;
          b2RevoluteJointDef revoluteJointDef52; 
	  revoluteJointDef52.bodyA = man1;
	  revoluteJointDef52.bodyB = manhand;
	  revoluteJointDef52.collideConnected = false;
	  revoluteJointDef52.localAnchorA.Set(4,0);//the top right corner of the box
	  revoluteJointDef52.localAnchorB.Set(0,0);//center of the circle
	  revoluteJointDef52.enableLimit = true;
	  revoluteJointDef52.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef52.upperAngle = -10* DEGTORAD;  
	  revoluteJoint52 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef52);
	  
	  //! elbow joint of rear man  
      b2RevoluteJoint* revoluteJoint522;
      b2RevoluteJointDef revoluteJointDef522; 
      revoluteJointDef522.bodyA = manhand;
      revoluteJointDef522.bodyB = m_bodyGA6;
      revoluteJointDef522.collideConnected = false;
      revoluteJointDef522.localAnchorA.Set(2,0);
      revoluteJointDef522.localAnchorB.Set(-2.5,0);
      revoluteJointDef522.enableLimit = true;
      revoluteJoint522 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef522); 
	 
	 //! spear's rod
	  b2BodyDef weaponDef;
	  b2FixtureDef weaponFix;     
      b2PolygonShape weaponPoly;;
	  weaponPoly.SetAsBox(4,0.1f);
	  weaponFix.shape=&weaponPoly;
	  weaponFix.restitution=0;
	  weaponDef.type = b2_dynamicBody;
	  weaponDef.position.Set(-2.5f,32);
	  weapon=m_world->CreateBody(&weaponDef);
      weapon->CreateFixture(&weaponFix);
	  weapon->SetTransform(weapon->GetPosition(),55*DEGTORAD);
	  weapon->SetAngularDamping(1);
	  
	  //! joint between rear man's hand and spear
      revoluteJointDef82.bodyA = m_bodyGA6;
      revoluteJointDef82.bodyB = weapon;
      revoluteJointDef82.collideConnected = false;
      revoluteJointDef82.localAnchorA.Set(2,0);
      revoluteJointDef82.localAnchorB.Set(-3,0);
      revoluteJointDef82.enableLimit = true;
      revoluteJointDef82.lowerAngle = 10* DEGTORAD;
      revoluteJointDef82.upperAngle = -10* DEGTORAD; 
      revoluteJoint82 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef82); 
	 
	  //! front man's head
	  b2Body *manhead;
	  b2BodyDef manheadDef;
	  b2FixtureDef manheadFix;
	  b2PolygonShape manheadPoly;;
	  manheadPoly.SetAsBox(2,2);
	  manheadFix.shape = &manheadPoly;
	  manheadFix.restitution = 0;
	  manheadDef.position.Set(-12, 34);
	  manheadDef.type = b2_dynamicBody;
	  manhead = m_world->CreateBody(&manheadDef);
	  manhead->CreateFixture(&manheadFix);
		
		
	  //!joint between front man's neck and head	 
      b2WeldJointDef jointDef3;
      jointDef3.bodyA = m_bodyF;
      jointDef3.bodyB = manhead;
      jointDef3.collideConnected = true;
      jointDef3.localAnchorA.Set(0,1);
	  jointDef3.localAnchorB.Set(0,-2);
	  joint3= (b2WeldJoint*)m_world->CreateJoint( &jointDef3 ); 
	       
	       
	   //! rear man's head
	  b2Body *manh1;
	  b2BodyDef manh1Def;
	  b2FixtureDef manh1Fix;
      b2PolygonShape manh1Poly;
	  manh1Poly.SetAsBox(2,2);
	  manh1Fix.shape = &manh1Poly;
	  manh1Def.position.Set(-12,35) ;
	  manh1Def.type = b2_dynamicBody;
	  manh1 = m_world->CreateBody(&manh1Def);
	  manh1->CreateFixture(&manh1Fix);
		
	 //!joit between rear man's head and his neck
	  b2RevoluteJoint* revoluteJoint0;
      b2RevoluteJointDef revoluteJointDef0; 
	  revoluteJointDef0.bodyA = manh;
	  revoluteJointDef0.bodyB = manh1;
	  revoluteJointDef0.collideConnected = false;
	  revoluteJointDef0.localAnchorA.Set(0,1);
	  revoluteJointDef0.localAnchorB.Set(0,-2);
	  revoluteJointDef0.enableLimit = false;
	  revoluteJointDef0.enableMotor=true;
	  revoluteJointDef0.motorSpeed=40.0f;
	  revoluteJointDef0.maxMotorTorque=100.0f;
	  revoluteJoint0 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef0);
		 
	 //! spear's corner tringle
	  b2BodyDef triDef;
	  b2FixtureDef triFix;
	  b2PolygonShape triPoly;
	  b2Vec2 vertices2[3];
	  vertices2[0].Set(-2,1.0f);    
	  vertices2[1].Set(0,0.5f);
	  vertices2[2].Set(-2,0);
	  triPoly.Set(vertices2, 3);
	  triFix.shape = &triPoly;
	  triDef.position.Set(2,38.2f);
	  triDef.type = b2_dynamicBody;
	  tri = m_world->CreateBody(&triDef);
	  tri->CreateFixture(&triFix);
	  tri->SetTransform(tri->GetPosition(),55*DEGTORAD);
	  tri->SetAngularDamping(1);
	
	  //! joint between spear rod and tringle	
	  b2RevoluteJoint* revoluteJoint821;
      b2RevoluteJointDef revoluteJointDef821; 
	  revoluteJointDef821.bodyA = weapon;
	  revoluteJointDef821.bodyB = tri;
	  revoluteJointDef821.collideConnected = false;
	  revoluteJointDef821.localAnchorA.Set(4,0);
	  revoluteJointDef821.localAnchorB.Set(-2,0.5f);
	  revoluteJointDef821.enableLimit = false;
	  revoluteJointDef821.lowerAngle = 10* DEGTORAD;
	  revoluteJointDef821.upperAngle = -10* DEGTORAD;	 
	  revoluteJoint821 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef821);
	 
	  //! moving box in sumilation
	  float distance;
	  distance=weapon->GetLinearVelocity().x;
	  b2PolygonShape mobjectPoly;
	  mobjectPoly.SetAsBox(1.5,5);	
	  mobjectFix.shape = &mobjectPoly;					
	  mobjectFix.friction = 0;					
	  mobjectDef.position.Set(distance+40,2) ;
	  mobjectDef.type = b2_dynamicBody;
	  mobject = m_world->CreateBody(&mobjectDef);
	  mobject->CreateFixture(&mobjectFix);
	  mobject->SetLinearVelocity( b2Vec2(10,0) );
}
		
		//! Keyboard function inherited and overriden from base_sim_t class
		void keyboard(unsigned char key)
	    { 
	    switch (key)
	    {
	      case 'q': //move left
	      for(int i=1; i<=5; i++)
	      {
			m_bodyGA5->SetAngularVelocity(-100);
			m_bodyG4->SetAngularVelocity(100);
		  }
	      break;
	      
	      case 'w': //move right
	        m_bodyC->SetAngularVelocity(-100* DEGTORAD);
	        m_bodyG->SetTransform(m_bodyG->GetPosition(),15*DEGTORAD);
	        m_bodyJ->SetTransform(m_bodyJ->GetPosition(),15*DEGTORAD);
	        m_bodyB->SetAngularVelocity(0);
	        m_bodyC->SetAngularVelocity(0);
	        m_bodyG1->SetLinearVelocity( b2Vec2(0,0) );
	        m_bodyC->SetLinearVelocity( b2Vec2(0,0) );
	        m_bodyB->SetLinearVelocity( b2Vec2(0,0) );
	        break;
	        m_bodyG1->SetLinearVelocity( b2Vec2(10,0) );
	        
	      case 'e': //stop
	      dummy1=true;
	      state=1;
	      x1=m_bodyG1->GetPosition().x;
	      for(int i=1; i<=5; i++)
	      {
			  state=1;
		  }
	      vel++;
	      cout<<m_bodyG2->GetPosition().x<<"\n";
	      for (int i=1; i<=250; i++)
	          {
	            m_bodyG6->SetTransform(m_bodyG->GetPosition(),-0.16*i*DEGTORAD);
	            m_bodyJ->SetTransform(m_bodyJ->GetPosition(),-0.26*i*DEGTORAD);
	          }
	          m_bodyG1->SetLinearVelocity(b2Vec2(m_bodyG1->GetLinearVelocity().x+30,0));
			  cout<<"Entered state e";
	        break;
				 
	      case 'j': //jump
				{
				m_world->DestroyJoint(revoluteJoint82);
				weapon->SetAngularVelocity(-1.5);
				tri->SetAngularVelocity(-1.5);
				weapon->SetLinearVelocity( b2Vec2(40,25) );
				m_bodyGA6->SetTransform(m_bodyGA6->GetPosition(),40*DEGTORAD);
		        break;
	           }
	      default:
	      //! run default behaviour
	      cs296::base_sim_t::keyboard(key);
	    }
	    
	  }
	    
	  //! step function inherited and overriden from base_sim_t class
      void step(cs296::settings_t* settings){
	  cs296::base_sim_t::step(settings);
	  if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/4%2==1)){
	       cout<<"1";
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (55)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-25)*DEGTORAD ); 
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (-25)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (0)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (55)*DEGTORAD );
	       cout<<m_bodyG1->GetPosition().x<<"\n";
	  }
	  if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/4%2==0)){
		   cout<<m_bodyG1->GetPosition().x;
		   revoluteJointDef73.lowerAngle=60*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-60*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=60*DEGTORAD;
		   revoluteJointDef7A2.lowerAngle=-60*DEGTORAD;
		   revoluteJointDef75.lowerAngle=30*DEGTORAD;
		   revoluteJointDef73.lowerAngle=-30*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=30*DEGTORAD;
		   revoluteJointDef7A4.lowerAngle=-30*DEGTORAD;
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (55)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-0)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-0)*DEGTORAD );
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-25)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (55)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (-25)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (0)*DEGTORAD );
	       state=3;
		}
  	  if(dummy1&&((int)(m_bodyG1->GetPosition().x-x1)/4%2==2)){
		   cout<<m_bodyG1->GetPosition().x;
	       m_bodyG3->SetTransform( m_bodyG3->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA3->SetTransform( m_bodyGA3->GetPosition(), (-0)*DEGTORAD );
	       m_bodyGA5->SetTransform( m_bodyGA5->GetPosition(), (-30)*DEGTORAD );
	       m_bodyG5->SetTransform( m_bodyG5->GetPosition(), (-30)*DEGTORAD );
	       m_bodyGA2->SetTransform( m_bodyGA2->GetPosition(), (0)*DEGTORAD );
	       m_bodyG2->SetTransform( m_bodyG2->GetPosition(), (0)*DEGTORAD );
	       m_bodyGA4->SetTransform( m_bodyGA4->GetPosition(), (40)*DEGTORAD );
	       m_bodyG4->SetTransform( m_bodyG4->GetPosition(), (40)*DEGTORAD );
	       m_bodyG1->SetLinearVelocity(b2Vec2(m_bodyG1->GetLinearVelocity().x+0.2,0));
	       state=3;
	    }
	  }
	  
	
	    
	    static cs296::base_sim_t* create()
	    {
	      return new dominos_t;
	    }
	   
	
	  
	};
	namespace cs296{ 
	  sim_t *sim = new sim_t("Dominos", dominos_t::create);
	}
	
#endif
