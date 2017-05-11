#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>
#include <sstream>
#include "RAstar_ros.h"

void spinThread(bool *cont) {
    ros::Rate loop_rate(10);

    while (ros::ok() && *cont) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


/**
 *   @brief  Test cells inside map
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testIsCellInsideMap) {
    
    RAstar_planner::RAstarPlannerROS plan;
    float x = 1.0;
    float y = 1.0;
    plan.width = 10;
    plan.height = 10;
    plan.resolution = 5;

    EXPECT_EQ(true, plan.isCellInsideMap(x,y));
}


/**
 *   @brief  Test getting coordinates
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testgetCoord) {
    
    RAstar_planner::RAstarPlannerROS plan;

    float x = 5.0;
    float y = 7.0;
    plan.originX = 2.0;
    plan.originY = 3.0;
    plan.getCorrdinate(x,y);

    EXPECT_EQ(3.0, x);
    EXPECT_EQ(4.0,y);
}


/**
 *   @brief  Test conversion to cell index
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testConvertToCellIndex) {
    
    RAstar_planner::RAstarPlannerROS plan;

    float x = 20.0;
    float y = 30.0;
    plan.resolution = 10.0;
    plan.width = 1.0;
    
    EXPECT_EQ(5, plan.convertToCellIndex(x,y));
}


/**
 *   @brief  Test conversion to coordinates
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testConvertToCoord) {
    
    RAstar_planner::RAstarPlannerROS plan;

    float x = 10.0;
    float y = 10.0;
    plan.resolution = 5.0;
    int index = 10;
    plan.originX = 2.0;
    plan.originY = 3.0; 
    plan.width = 5.0;
    plan.convertToCoordinate(index,x,y);
    EXPECT_EQ(2, x);
    EXPECT_EQ(13,y);
}


/**
 *   @brief  Test finding free neighbors
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testFindFreeNeighborCell) {
    
    RAstar_planner::RAstarPlannerROS plan;
    plan.width = 3.0;
    plan.height = 3.0;
    int id = 4;
    int mapSize = plan.width*plan.height;
    plan.OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < plan.height; iy++)
    {
      for (unsigned int ix = 0; ix < plan.width; ix++)
      {
       int cost = 0;
        
        if (cost == 0) {
          plan.OGM[iy*plan.width+ix]=true;
	 }
       }
    }
    
    int ints[] = {0,1,2,3,5,6,7,8};
    vector <int> test (ints, ints+sizeof(ints)/sizeof(int));
    vector <int> nay;
    nay =  plan.findFreeNeighborCell(id);
    EXPECT_EQ(test, nay);   
}


/**
 *   @brief  Test finding path
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testFindPath) {
    
    RAstar_planner::RAstarPlannerROS plan;
    plan.width = 5.0;
    plan.height = 5.0;
    //plan.width2 = 10.0;
    //plan.height2 = 10.0;
    float infinity = std::numeric_limits< float >::infinity();
    int startCell = 1;
    int endCell = 23;
    int mapSize = plan.width*plan.height;
    //plan.OGM;
    plan.OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < plan.height; iy++)
    {
      for (unsigned int ix = 0; ix < plan.width; ix++)
      {
       int cost = 0;
        
        if (cost == 0) {
          plan.OGM[iy*plan.width+ix]=true;
	  //ROS_INFO_STREAM("OGM["<<iy*plan.width+ix<<"] =" << plan.OGM[iy*plan.width+ix]);
	}
       }
    }
    //ROS_INFO_STREAM("OGM[0] = " << plan.OGM[0]);
    float *g_score = new float[mapSize];
    for (uint i=0; i<mapSize; i++)
	g_score[i]=infinity;
    
    int ints[] = {1,6,11,17,23};
    vector <int> test (ints, ints+sizeof(ints)/sizeof(int));
    vector<int> bpath;
    //ROS_INFO("bpath1");
    bpath =  plan.findPath(startCell,endCell,g_score);
    //ROS_INFO("bpath2");
    EXPECT_EQ(test, bpath);
}


/**
 *   @brief  Test caluclating Heuristic
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testCalcH) {
    
    RAstar_planner::RAstarPlannerROS plan;
    int cell = 10;
    int goal = 30;
    float dist = 4.0;
    
    int cell2 = 10;
    int goal2 = 33;
    float dist2 = 7.0;
    plan.width = 5.0;
    float H = plan.calculateHCost(cell,goal);
    float H2 = plan.calculateHCost(cell2,goal2);
    EXPECT_EQ(dist, H);    
    EXPECT_EQ(dist2, H2);
}


/**
 *   @brief  Test adding cell to open list
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testAddNToCellOpenList) {
    
    RAstar_planner::RAstarPlannerROS plan;
    int neighborCell = 1;
    int goalCell = 3;
    plan.width = 5.0;
    plan.height = 5.0;
    int mapSize = plan.width*plan.height;
    float infinity = std::numeric_limits< float >::infinity();
    float *g_score = new float[mapSize];
    for (uint i=0; i<mapSize; i++)
	g_score[i]=5;
    multiset<cells> OPL;
    
    EXPECT_EQ(true, OPL.empty());  
    plan.addNeighborCellToOpenList(OPL,neighborCell,goalCell,g_score);
    EXPECT_EQ(false, OPL.empty());    
}


/**
 *   @brief  Test start/goal locations being valid
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testIsStartGoalValid) {
    
    RAstar_planner::RAstarPlannerROS plan;
    int start1 = 10;
    int goal1 = 10;

    int start2 = 90;
    int goal2 = 91;

    int start3 = 90;
    int goal3 = 10;

    int start4 = 10;
    int goal4 = 90;

    int start5 = 10;
    int goal5 = 11;

    plan.width = 5;
    plan.height = 5;
    int mapSize = plan.width*plan.height;
    plan.OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < plan.height; iy++)
    {
      for (unsigned int ix = 0; ix < plan.width; ix++)
      {
       int cost = 0;
        
        if (cost == 0) {
          plan.OGM[iy*plan.width+ix]=true;
	}
       }
    }
   
    EXPECT_EQ(false,plan.isStartAndGoalCellsValid(start1,goal1) );
    EXPECT_EQ(false,plan.isStartAndGoalCellsValid(start2,goal2) );
    EXPECT_EQ(false,plan.isStartAndGoalCellsValid(start3,goal3) );
    EXPECT_EQ(false,plan.isStartAndGoalCellsValid(start4,goal4) );
    EXPECT_EQ(true ,plan.isStartAndGoalCellsValid(start5,goal5) );
   
    
}


/**
 *   @brief  Test getting move costs
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testGetMoveCost) {
    
    RAstar_planner::RAstarPlannerROS plan;
    int cell1 = 1;
    int cell2 = 2;
    int cell3 = 5;
    plan.width = 5.0;
    float d1 = 1;
    float d2 = 1.4;
    EXPECT_EQ(d1, plan.getMoveCost(cell1,cell2));  
    EXPECT_EQ(0, d2 - plan.getMoveCost(cell1,cell3));  
    }



/**
 *   @brief  Test check if neighbor free
 *
 *   @param  none
 *   @return none
*/
TEST(TestPlannerl, testIsFree) {
    
    RAstar_planner::RAstarPlannerROS plan;
    int id = 1;
    int i = 2;
    int j = 5;
    plan.width = 5.0;
    plan.height = 5.0;
    int mapSize = plan.width*plan.height;
    plan.OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < plan.height; iy++)
    {
      for (unsigned int ix = 0; ix < plan.width; ix++)
      {
       int cost = 0;
        
        if (cost == 0) {
          plan.OGM[iy*plan.width+ix]=true;
	}
       }
    }
    plan.OGM[id] = false;
    EXPECT_EQ(true, plan.isFree(i,j));  
    EXPECT_EQ(false,plan.isFree(id));  
    }



/*
 *   @brief  unit test entrypoint
 *  
 *   @param  number of arguments
 *   @param  argument character array
 *   @return integer 0 upon exit success
*/
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testAstar");
    ros::NodeHandle nh;

    bool cont = true;

    // spawn another thread
    boost::thread th(spinThread, &cont);

    int ret = RUN_ALL_TESTS();

    cont = false;

    // wait the second thread to finish
    th.join();

    return ret;
}
