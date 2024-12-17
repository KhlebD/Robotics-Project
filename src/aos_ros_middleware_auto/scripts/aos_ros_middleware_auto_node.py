#!/usr/bin/python3
import datetime
import rospy  
import pymongo
import operator
import traceback
from std_msgs.msg import Int8,Bool,String
from task3_env.srv import *

 
DEBUG = True
HEAVY_LOCAL_VARS={}
aosDbConnection = pymongo.MongoClient("mongodb://localhost:27017/")
aosDB = aosDbConnection["AOS"]
aos_statisticsDB = aosDbConnection["AOS_Statistics"]
aos_local_var_collection = aosDB["LocalVariables"]
aosStats_local_var_collection = aos_statisticsDB["LocalVariables"]
aos_GlobalVariablesAssignments_collection = aosDB["GlobalVariablesAssignments"]
aos_ModuleResponses_collection = aosDB["ModuleResponses"]
collActionForExecution = aosDB["ActionsForExecution"]
collErros = aosDB["Errors"]
collLogs = aosDB["Logs"]
collActions = aosDB["Actions"]

def registerError(errorStr,trace, comments=None):
    error = {"Component": "aosRosMiddleware", "Error": errorStr,"Trace":trace,
             "Time": datetime.datetime.utcnow()}
    if comments is not None:
        error = {"Component": "aosRosMiddleware", "Error": errorStr, "Trace": trace,
                 "Time": datetime.datetime.utcnow(), "Comments":comments}
    collErros.insert_one(error)


def registerLog(str):
    error = {"Component": "aosRosMiddleware", "Event": str,
             "Time": datetime.datetime.utcnow()}
    collLogs.insert_one(error)

def getHeavyLocalVarList(moduleName):
    if moduleName in HEAVY_LOCAL_VARS:
        return HEAVY_LOCAL_VARS[moduleName]
    else:
        return []



class ListenToMongoDbCommands:
    def __init__(self, _topicListener):
        self.currentActionSequenceID = 1
        self.currentActionFotExecutionId = None
        self._topicListener = _topicListener
        self.readyToActivate = "" 
        self.placeServiceName = "/place"
        self.navigateServiceName = "/navigate"
        self.pickServiceName = "/pick"

        self.listen_to_mongodb_commands()



    def handle_place(self, params):
        responseNotByLocalVariables = None
        try:
            registerLog("wait for service: moduleName:place, serviceName:place")
            rospy.wait_for_service(self.placeServiceName)
            place_proxy = rospy.ServiceProxy(self.placeServiceName, place)
            registerLog("Sending request to service, moduleName:place")
            __input = place_proxy()
            registerLog("Service response received, moduleName:place")
            skill_success_place = __input.success
            self._topicListener.updateLocalVariableValue("skill_success_place",skill_success_place)
            if DEBUG:
                print("place service terminated")
        except Exception as e:
            registerError(str(e),traceback.format_exc(e),'Action: place')
            print("Service call failed")
        
        return responseNotByLocalVariables
    def handle_navigate(self, params):
        responseNotByLocalVariables = None
        move_to_location = ""
        try:
            move_to_location = params["ParameterValues"]["oLocation"]
            self._topicListener.updateLocalVariableValue("move_to_location", move_to_location)
        except Exception as e:
            registerError(str(e), traceback.format_exc(e), 'Action: navigate, illegalActionObs')
            responseNotByLocalVariables = "illegalActionObs"
        try:
            registerLog("wait for service: moduleName:navigate, serviceName:navigate")
            rospy.wait_for_service(self.navigateServiceName)
            navigate_proxy = rospy.ServiceProxy(self.navigateServiceName, navigate)
            registerLog("Sending request to service, moduleName:navigate")
            __input = navigate_proxy(location=(move_to_location))
            registerLog("Service response received, moduleName:navigate")
            skill_success_navigate = __input.success
            self._topicListener.updateLocalVariableValue("skill_success_navigate",skill_success_navigate)
            if DEBUG:
                print("navigate service terminated")
        except Exception as e:
            registerError(str(e),traceback.format_exc(e),'Action: navigate')
            print("Service call failed")
        
        return responseNotByLocalVariables
    def handle_pick(self, params):
        responseNotByLocalVariables = None
        pick_toy = ""
        try:
            pick_toy = params["ParameterValues"]["oColor"]
            self._topicListener.updateLocalVariableValue("pick_toy", pick_toy)
        except Exception as e:
            registerError(str(e), traceback.format_exc(e), 'Action: pick, illegalActionObs')
            responseNotByLocalVariables = "illegalActionObs"
        try:
            registerLog("wait for service: moduleName:pick, serviceName:pick")
            rospy.wait_for_service(self.pickServiceName)
            pick_proxy = rospy.ServiceProxy(self.pickServiceName, pick)
            registerLog("Sending request to service, moduleName:pick")
            __input = pick_proxy(toy_type=(pick_toy))
            registerLog("Service response received, moduleName:pick")
            skill_success_pick=__input.success
            self._topicListener.updateLocalVariableValue("skill_success_pick",skill_success_pick)
            if DEBUG:
                print("pick service terminated")
        except Exception as e:
            registerError(str(e),traceback.format_exc(e),'Action: pick')
            print("Service call failed")
        
        return responseNotByLocalVariables



    def saveHeavyLocalVariableToDB(self, moduleName):
        for varName in getHeavyLocalVarList(moduleName):
            value = self._topicListener.localVarNamesAndValues[moduleName][varName]
            aos_local_var_collection.replace_one({"Module": moduleName, "VarName": varName},
                                                 {"Module": moduleName, "VarName": varName,
                                                  "Value": value}, upsert=True)
            aosStats_local_var_collection.insert_one(
                {"Module": moduleName, "VarName": varName, "value": value,
                 "Time": datetime.datetime.utcnow()})




    def registerModuleResponse(self, moduleName, startTime, actionSequenceID, responseNotByLocalVariables):
        self.saveHeavyLocalVariableToDB(moduleName)
        filter1 = {"ActionSequenceId": actionSequenceID}
        if DEBUG:
            print("registerModuleResponse()")

        if responseNotByLocalVariables is not None:
            moduleResponseItem = {"Module": moduleName, "ActionSequenceId": actionSequenceID,
                                  "ModuleResponseText": responseNotByLocalVariables, "StartTime": startTime,
                                  "EndTime": datetime.datetime.utcnow(),
                                  "ActionForExecutionId": self.currentActionFotExecutionId}
            #aos_ModuleResponses_collection.insert_one(moduleResponseItem)
            aos_ModuleResponses_collection.replace_one(filter1,moduleResponseItem, upsert=True)
            return

        moduleResponse = ""
        assignGlobalVar = {}
        if moduleName == "place":
            skill_success_place = self._topicListener.localVarNamesAndValues["place"]["skill_success_place"]
            if DEBUG:
                print("place action local variables:")
                print("skill_success_place:")
                print(skill_success_place)
            if moduleResponse == "" and skill_success_place:
                moduleResponse = "place_eSuccess"
            if moduleResponse == "" and True:
                moduleResponse = "place_eFailed"

        if moduleName == "navigate":
            skill_success_navigate = self._topicListener.localVarNamesAndValues["navigate"]["skill_success_navigate"]
            move_to_location = self._topicListener.localVarNamesAndValues["navigate"]["move_to_location"]
            if DEBUG:
                print("navigate action local variables:")
                print("skill_success_navigate:")
                print(skill_success_navigate)
            if moduleResponse == "" and skill_success_navigate:
                moduleResponse = "navigate_eSuccess"

        if moduleName == "pick":
            skill_success_pick = self._topicListener.localVarNamesAndValues["pick"]["skill_success_pick"]
            pick_toy = self._topicListener.localVarNamesAndValues["pick"]["pick_toy"]
            if DEBUG:
                print("pick action local variables:")
                print("skill_success_pick:")
                print(skill_success_pick)
            if moduleResponse == "" and skill_success_pick:
                moduleResponse = "pick_eSuccess"
            if moduleResponse == "" and True:
                moduleResponse = "pick_eFailed"



        if DEBUG and len(getHeavyLocalVarList(moduleName)) == 0:
            print("moduleResponse result:")
            print(moduleResponse)
        moduleLocalVars = {}
        if moduleName in self._topicListener.localVarNamesAndValues.keys():
            moduleLocalVars=self._topicListener.localVarNamesAndValues[moduleName]
        moduleResponseItem = {"Module": moduleName, "ActionSequenceId": actionSequenceID,
                "ModuleResponseText": moduleResponse, "StartTime": startTime, "EndTime": datetime.datetime.utcnow(), "ActionForExecutionId":self.currentActionFotExecutionId, 
                "LocalVariables":moduleLocalVars}


        #aos_ModuleResponses_collection.insert_one(moduleResponseItem)
        aos_ModuleResponses_collection.replace_one(filter1,moduleResponseItem, upsert=True)
        for varName, value in assignGlobalVar.items():
            isInit = False
            if value is not None:
                isInit = True
            aos_GlobalVariablesAssignments_collection.replace_one({"GlobalVariableName": varName},
                                                                  {"GlobalVariableName": varName, "LowLevelValue": value,
                                                                   "IsInitialized": isInit, "UpdatingActionSequenceId":actionSequenceID,
                                                                   "ModuleResponseId":moduleResponseItem["_id"]}, upsert=True)

    def listen_to_mongodb_commands(self):
        while(True):
            filter1 = {"ActionSequenceId": self.currentActionSequenceID}
            actionForExecution = collActionForExecution.find_one(filter1)
            if(actionForExecution is not None):
                if DEBUG:
                    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                    print("actionForExecution:")
                    print(actionForExecution)
                    print("actionID:")
                    print(actionForExecution["ActionID"])
                moduleName = actionForExecution["ActionName"]
                actionParameters = actionForExecution["Parameters"] 
                self.currentActionFotExecutionId = actionForExecution["_id"]
                self._topicListener.setListenTarget(moduleName)
                rospy.sleep(0.3)#0.03 is a tested duration, not to drop updates
                moduleActivationStartTime = datetime.datetime.utcnow()
                responseNotByLocalVariables = None
                print("module name:")
                print(moduleName)
                registerLog("Request to call to module:"+moduleName)
                
                if moduleName == "pick":
                    print("handle pick")
                    responseNotByLocalVariables = self.handle_pick(actionParameters)
                if moduleName == "place":
                    print("handle place")
                    responseNotByLocalVariables = self.handle_place(actionParameters)
                if moduleName == "navigate":
                    print("handle navigate")
                    responseNotByLocalVariables = self.handle_navigate(actionParameters)

                rospy.sleep(0.3)#0.015 is a tested duration, not to drop updates
                self._topicListener.setListenTarget("after action")

                self.registerModuleResponse(moduleName, moduleActivationStartTime, self.currentActionSequenceID, responseNotByLocalVariables)
                if DEBUG:
                    print("self.currentActionSequenceID:")
                    print(self.currentActionSequenceID)
                self.currentActionSequenceID = self.currentActionSequenceID +1
                if DEBUG:
                    print("self.currentActionSequenceID:")
                    print(self.currentActionSequenceID)
                self.currentActionFotExecutionId = None
            rospy.sleep(0.1)

class AOS_TopicListenerServer:
    def __init__(self):
        self.localVarNamesAndValues = {"place":{"skill_success_place": None}, "navigate":{"skill_success_navigate": None, "move_to_location": None}, "pick":{"skill_success_pick": None, "pick_toy": None}}
        self.setListenTarget("initTopicListener")


    def initLocalVars(self, moduleNameToInit):
        if DEBUG:
            print("initLocalVars:")
            print(moduleNameToInit)
        for moduleName, localVarNamesAndValuesPerModule in self.localVarNamesAndValues.items():
            for localVarName, value in localVarNamesAndValuesPerModule.items():
                if moduleName == moduleNameToInit:
                    if DEBUG:
                        print ("init var:")
                        print(localVarName)
                    aos_local_var_collection.replace_one({"Module": moduleName, "VarName": localVarName},
                                                         {"Module": moduleName, "VarName": localVarName, "Value": value},
                                                         upsert=True)
                    aosStats_local_var_collection.insert_one(
                        {"Module": moduleName, "VarName": localVarName, "value": value, "Time": datetime.datetime.utcnow()})


    def setListenTarget(self, _listenTargetModule):
        self.initLocalVars(_listenTargetModule)
        if DEBUG:
            print('setListenTopicTargetModule:')
            print(_listenTargetModule)
        self.listenTargetModule = _listenTargetModule


    def updateLocalVariableValue(self, varName, value):
        if DEBUG and varName not in getHeavyLocalVarList(self.listenTargetModule):
            print("update local var:")
            print(varName)
            print(value)
        if self.listenTargetModule not in self.localVarNamesAndValues:
            return
        if self.localVarNamesAndValues[self.listenTargetModule][varName] != value:
            if DEBUG:
                print("ACTUAL UPDATE --------------------------------------------------------------------------")
            self.localVarNamesAndValues[self.listenTargetModule][varName]=value
            if varName not in getHeavyLocalVarList(self.listenTargetModule):
                aos_local_var_collection.replace_one({"Module": self.listenTargetModule, "VarName":varName}, {"Module": self.listenTargetModule, "VarName":varName, "Value":value}, upsert=True)
                aosStats_local_var_collection.insert_one(
                    {"Module": self.listenTargetModule, "VarName": varName, "value": value, "Time": datetime.datetime.utcnow()})
                if DEBUG:
                    print("WAS UPDATED --------------------------------------------------------------------------")







class AOS_InitEnvironmentFile:
    def __init__(self):
        pass

    def updateGlobalVarLowLevelValue(self, varName, value):
        isInit = value is not None
        aos_GlobalVariablesAssignments_collection.replace_one({"GlobalVariableName": varName},{"GlobalVariableName": varName, "LowLevelValue": value,
                                                                                               "IsInitialized": isInit, "UpdatingActionSequenceId": "initialization",
                                                                                               "ModuleResponseId": "initialization"},upsert=True)








if __name__ == '__main__':
    try:
        rospy.init_node('aos_ros_middleware_auto', anonymous=True)
        AOS_InitEnvironmentFile()
        topicListener = AOS_TopicListenerServer()
        commandlistener = ListenToMongoDbCommands(topicListener)
    except Exception as e:
        registerError(str(e), traceback.format_exc(e))
    