//
// Created by carlotta on 19/05/17.
//

#ifndef MEMORY_MANAGERS_MEMORYMANAGER_H
#define MEMORY_MANAGERS_MEMORYMANAGER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <ros/console.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "pitt_msgs/TrackedShapes.h"
#include "pitt_msgs/TrackedShape.h"
#include "sit_msgs/EpisodicInterface.h"
#include "sit_msgs/SemanticInterface.h"
#include "sit_msgs/ScoreInterface.h"
#include <vector>


using namespace ros;
using namespace std;
using namespace pitt_msgs;
using namespace sensor_msgs;
using namespace sit_msgs;



namespace memManager{

        //node names
        static const string  MEMORIZATION_MANAGER_NODE_NAME= "MemorizationManager";
        //srv names
        static const string SRV_SEMANTIC="SemanticService";
        static const string SRV_EPISODIC="EpisodicService";
        static const string SRV_SCORE="ScoreService";
        //names --> who is publishing
        //topic names
        //input manager
        static const string MEMORIZATION_ACTIVATION="memory_managers/inputManager/memorizationActivation";

        //memorizationManager
        static const string MEMORIZATION_EPISODIC_MEMORY_TOPIC="memory_managers/memorizationManager/memorizationTopic/EpisodicMemory";
        static const string MEMORIZATION_SEMANTIC_MEMORY_TOPIC="memory_managers/memorizationManager/memorizationTopic/SemanticMemory";
        static const string PITT_TOPIC="ransac_segmentation/trackedShapes";
        //output manager
        static const string INPUT_MANAGER_ACTIVATION="memory_managers/outputManager/inputManagerActivation";
        //user input
        static const string POSITIVE_RESPONSE="Y";
        static const string NEGATIVE_RESPONSE="N";
        //************SIT ONTOLOGY*********//
        //data  properties names
        static const string PROPR_HAS_GEOMETRIC_AXISX="has-geometric_axisX";
        static const string PROPR_HAS_GEOMETRIC_AXISY="has-geometric_axisY";
        static const string PROPR_HAS_GEOMETRIC_AXISZ="has-geometric_axisZ";
        static const string PROPR_HAS_GEOMETRIC_CENTERX="has-geometric_centerX";
        static const string PROPR_HAS_GEOMETRIC_CENTERY="has-geometric_centerY";
        static const string PROPR_HAS_GEOMETRIC_CENTERZ="has-geometric_centerZ";
        static const string PROPR_HAS_GEOMETRIC_HEIGHT="has-geometric_height";
        static const string PROPR_CONE_HEIGHT="has-cone_height";
        static const string PROPR_CYLINDER_HEIGHT="has-cylinder_height";
        static const string PROPR_HAS_GEOMETRIC_HESSIAN="has-geometric_hessian";
        static const string PROP_HAS_GEOMETRIC_POINTX="has-geometric_pointX";
        static const string PROP_HAS_CONE_APEXX="has-cone_apexX";
        static const string PROP_HAS_CYLINDER_POINTX="has-cylinder_pointX";
        static const string PROP_HAS_GEOMETRIC_POINTY="has-geometric_pointY";
        static const string PROP_HAS_CONE_APEXY="has-cone_apexY";
        static const string PROP_HAS_CYLINDER_POINTY="has-cylinder_pointY";
        static const string PROP_HAS_GEOMETRIC_POINTZ="has-geometric_pointZ";
        static const string PROP_HAS_CONE_APEXZ="has-cone_apexZ";
        static const string PROP_HAS_CYLINDER_POINTZ="has-cylinder_pointZ";
        static const string PROP_HAS_GEOMETRIC_RADIUS ="has-geometric_radius";
        static const string PROP_HAS_CONE_RADIUS ="has-cone_radius";
        static const string PROP_HAS_CYLINDER_RADIUS ="has-cylinder_radius";
        static const string PROP_HAS_SPHERE_RADIUS ="has-sphere_radius";
        static const string PROP_HAS_ID="has_id";
        static const string PROP_HAS_TIME="has_time";

        //object  properties names
        static const string PROP_HAS_SCENE_ABOVE="has-scene_above";
        static const string PROP_HAS_SCENE_ALONGX="has-scene_alongX";
        static const string PROP_HAS_SCENE_ALONGY="has-scene_alongY";
        static const string PROP_HAS_SCENE_ALONGZ="has-scene_alongZ";
        static const string PROP_HAS_SCENE_BEHIND="has-scebe_behind";
        static const string PROP_HAS_SCENE_BLUE="has-scene_blue";
        static const string PROP_HAS_SCENE_COAXIAL="has-scene_coaxial";
        static const string PROP_HAS_SCENE_GREEN="has-scene_green";
        static const string PROP_HAS_SCENE_PARALLEL="has-scene_parallel";
        static const string PROP_HAS_SCENE_PERPENDICULAR="has-scene_perpendicular";
        static const string PROP_HAS_SCENE_PINK="has-scene_pink";
        static const string PROP_HAS_SCENE_RED="has-scene_red";
        static const string PROP_HAS_SCENE_RIGHT="has-scene_right";
        static const string PROP_HAS_SCENE_SUPPORT="has-scene_support";
        static const string PROP_HAS_SCENE_YELLOW="has-scene_yellow";
        static const string PROP_IS_ON_SUPPORT="is-on-support";
        static const string PROP_IS_SUPPORT_OF_PRIMITIVE="is-support-of_primitive";
        static const string PROP_IS_SUPPORT_OF_SCENE="is-support-of_scene";
        static const string PROP_IS_ABOVE_OF="isAboveOf";
        static const string PROP_IS_ALONG_X="isAlongX";
        static const string PROP_IS_ALONG_Y="isAlongY";
        static const string PROP_IS_ALONG_Z="isAlongZ";
        static const string PROP_IS_BEHIND_OF="isBehindOf";
        static const string PROP_IS_BELOW_OF="isBelowOf";
        static const string PROP_IS_COAXIAL_WITH="isCoaxialWith";
        static const string PROP_IS_IN_FRONT_OF="isInFrontOf";
        static const string PROP_IS_LEFT_OF="isLeftOf";
        static const string PROP_IS_PARALLEL_TO="isParallelTo";
        static const string PROP_IS_PERPENDICULAR_TO="isPerpendicularTo";
        static const string PROP_IS_RIGHT_OF="isRightOf";
        //classes
        static const string SPHERE="Sphere";
        static const string CYLINDER="Cylinder";
        static const string CONE="Cone";
        static const string PLANE="Plane";
        static const string SCENE="Scene";
        static const string FORGOTTEN_SCENE="ToBeForgotten";
        static const string SUPPORT ="Support";
        //************SCORE ONTOLOGY*********//
        //data Property
        static const string SCORE_PROP_HAS_TIME="hasTime";
        static const string SCORE_PROP_HAS_VALUE="hasValue";
        static const string SCORE_PROP_NUMBER_BELONGING_INDIVIDUAL="NumberBelongingIndividual";
        static const string SCORE_PROP_NUMBER_EPISODIC_RETRIEVAL="NumberEpisodicRetrieval";
        static const string SCORE_PROP_NUMBER_RETRIEVAL="NumberRetrieval";
        static const string SCORE_PROP_NUMBER_SEMANTIC_RETRIEVAL="NumberSemanticRetrieval";
        static const string SCORE_PROP_NUMBER_SUB_CLASSES="NumberSubClasses";
        static const string SCORE_PROP_SCORE_BELONGING_INDIVIDUAL="ScoreBelongingIndividual";
        static const string SCORE_PROP_SCORE_SUB_CLASSES="ScoreSubClasses";
        static const string SCORE_PROP_TIMES_FORGOTTEN="TimesForgotten";
        static const string SCORE_PROP_TIMES_LOW_SCORE="TimesLowScore";
        static const string SCORE_PROP_USER_NO_FORGET="UserNoForget";
        //Object Property
        static const string SCORE_PROP_HAS_SCORE="hasScore";
        static const string SCORE_PROP_IS_SCORE_OF="isScoreOf";
        //Classes
        static const string SCORE_CLASS_SCENE="Scene";
        static const string SCORE_CLASS_SCORE="Score";
        static const string SCORE_CLASS_EPISODIC_SCORE="EpisodicScore";
        static const string SCORE_CLASS_SEMANTIC_SCORE="SemanticScore";
        static const string SCORE_CLASS_TOTAL_EPISODIC_SCORE="TotalEpisodicScore";
        static const string SCORE_CLASS_TOTAL_SEMANTIC_SCORE="TotalSemanticScore";
        static const string SCORE_CLASS_HIGH_SCORE="ScoreHigh";
        static const string SCORE_CLASS_EPISODIC_HIGH_SCORE="EpisodicScoreHigh";
        static const string SCORE_CLASS_SEMANTIC_HIGH_SCORE="SemanticScoreHigh";
        static const string SCORE_CLASS_LOW_SCORE="ScoreLow";
        static const string SCORE_CLASS_EPISODIC_LOW_SCORE="EpisodicScoreLow";
        static const string SCORE_CLASS_SEMANTIC_LOW_SCORE="SemanticScoreLow";
        static const string SCORE_CLASS_TO_BE_FORGOTTEN="ToBeForgotten";
        static const string SCORE_CLASS_EPISODIC_TO_BE_FORGOTTEN="EpisodicToBeForgotten";
        static const string SCORE_CLASS_SEMANTIC_TO_BE_FORGOTTEN="SemanticToBeForgotten";
        //individuals
        static const string SCORE_INDIVIDUAL_TOTAL_EPISODIC="totalEpisodic";
        static const string SCORE_INDIVIDUAL_TOTAL_SEMANTIC="totalSemantic";
        //prefix
        static const string SCENE_SPATIAL_PRFIX = "has-scene_";

        bool userCheck(){
            string input = "";
            while (true){
                cout << "Are the information correct (Y/N) ";
                getline(cin, input);
                if (input == POSITIVE_RESPONSE) {

                    return true ;
                }
                else if (input==NEGATIVE_RESPONSE){
                    return false;
                    }
                    else
                    {
                        cout<<"invalid input"<<endl;
                    }

            }

        }

    bool userCheckContinue (string info){
        string input = "";
        while (true){
            cout << "Do you want continue " << info<< " ? (Y/N)";
            getline(cin, input);
            if (input == POSITIVE_RESPONSE) {

                return true ;
            }
            else if (input==NEGATIVE_RESPONSE){
                return false;
            }
            else
            {
                cout<<"invalid input"<<endl;
            }

        }

    }

    bool userCheckValues(string info ){
        string input = "";
        while (true){
            cout << "Do you want to change also the "<<info<<" ? (Y/N) ";
            getline(cin, input);
            if (input == POSITIVE_RESPONSE) {
                return true ;
            }
            else if (input==NEGATIVE_RESPONSE){
                return false;
            }
            else
            {
                cout<<"invalid input"<<endl;
            }

        }

    }
    int userDecisionRetrieval(){

            while (true){
                string input="";
                cout<<"What do you want to do ?\n- 1 Semantic Retrieval;\n- 2 Episodic store items;\n";
                getline(cin,input);
                if (input=="1"){
                    return 1;
                }
                else if (input=="2"){
                    return 2;
                }
                else {
                    cout<<"value not included in the interval,\n insert an integer value equal either to 1 or  2  or 3, \n please redo selection";
                }

            }

    };
        int  userDecision(){

            while (true){
                string input="";
                cout<<"What do you want to do ?\n- 1 Memorize new items;\n- 2 Retrieve store items; \n- 3 Force Forgetting\n,-4 Recognition\n";
                getline(cin,input);
                if (input=="1"){
                    return 1;
                }
                else if (input=="2"){
                    return 2;
                }
                else if (input=="3"){
                    return 3;
                }

                else if (input=="4"){
                    return 4;
                }

                else {
                    cout<<"value not included in the interval,\n insert an integer value equal either to 1 or  2  or 3, \n please redo selection";
                }

            }

        }
    int  userDecisionForgetting(){

        while (true){
            string input="";
            cout<<"What do you want to do ?\n- 1 Check item status;\n- 2 Force Forgetting of an Item ; \n- 3 Prevent an item from being forgetting\n- 4 Remove user no forget "
                    "from an item "<<endl;
            getline(cin,input);
            if (input=="1"){
                return 1;
            }
            else if (input=="2"){
                return 2;
            }
            else if (input=="3"){
                return 3;
            }
            else if (input=="4"){
                return 4;
            }

            else {
                cout<<"value not included in the interval,\n insert an integer value equal either to 1 or  2  or 3, \n please redo selection";
            }

        }

    }
    bool userMemorizationDecision(){
        string input = "";
        while (true){
            cout << "Do you want to memorize other items (Y/N) ";
            getline(cin, input);
            if (input == POSITIVE_RESPONSE) {

                return true ;
            }
            else if (input==NEGATIVE_RESPONSE){
                return false;
            }
            else
            {
                cout<<"invalid input"<<endl;
            }

        }

    }
    bool userRetrievalAdd(string info ){
        string input = "";
        while (true){
            cout << "Do you want to add "<<info<<" (Y/N)?"<<endl;
            getline(cin, input);
            if (input == POSITIVE_RESPONSE) {

                return true ;
            }
            else if (input==NEGATIVE_RESPONSE){
                return false;
            }
            else
            {
                cout<<"invalid input"<<endl;
            }

        }

    }

    vector <float> coefficientsFromRansacToSemantic(TrackedShape shape){
        vector <float> coefficientsRansac;
        coefficientsRansac.push_back(shape.x_pc_centroid);
        coefficientsRansac.push_back(shape.y_pc_centroid);
        coefficientsRansac.push_back(shape.z_pc_centroid);
        if(shape.shape_tag==SPHERE){
            coefficientsRansac.push_back(shape.coefficients[3]);
        }
        else if (shape.shape_tag==CYLINDER){
            coefficientsRansac.push_back(shape.coefficients[0]);
            coefficientsRansac.push_back(shape.coefficients[1]);
            coefficientsRansac.push_back(shape.coefficients[2]);
            coefficientsRansac.push_back(shape.coefficients[3]);
            coefficientsRansac.push_back(shape.coefficients[4]);
            coefficientsRansac.push_back(shape.coefficients[5]);
            coefficientsRansac.push_back(shape.coefficients[6]);
            //height of the cylinder since it is not given
            cout<<"insert value for height cyilinder"<<endl;
            float height;
            cin>>height;
            coefficientsRansac.push_back(height);
        }
        else if (shape.shape_tag==CONE){
            coefficientsRansac.push_back(shape.coefficients[0]);
            coefficientsRansac.push_back(shape.coefficients[1]);
            coefficientsRansac.push_back(shape.coefficients[2]);
            coefficientsRansac.push_back(shape.coefficients[3]);
            coefficientsRansac.push_back(shape.coefficients[4]);
            coefficientsRansac.push_back(shape.coefficients[5]);
            //radius
            cout<<"insert value for radius cone"<<endl;
            float radius;
            cin>>radius;
            coefficientsRansac.push_back(radius);
            //height
            cout<<"insert value for height cone"<<endl;
            float height;
            cin>>height;
            coefficientsRansac.push_back(height);
        }
        else if (shape.shape_tag==PLANE){
            coefficientsRansac.insert(coefficientsRansac.end(),shape.coefficients.begin(),shape.coefficients.end());
        }

        return coefficientsRansac;


    }
    string colorRetrieval(string color){
        return SCENE_SPATIAL_PRFIX+color;
    };
    string spatialRelRetrieval(string spatialRelation){
        return SCENE_SPATIAL_PRFIX+spatialRelation;
    };
    void printCounterInfo(sit_msgs::ScoreCounter s , string counterType){
        cout<<"name of the item "<<s.scoreName<<endl;
        cout<<"value of the score "<<s.scoreValue<<endl;
        cout<<"value of the "<<counterType<<" counter"<<s.counter<<endl;
    };

}





#endif //MEMORY_MANAGERS_MEMORYMANAGER_H
