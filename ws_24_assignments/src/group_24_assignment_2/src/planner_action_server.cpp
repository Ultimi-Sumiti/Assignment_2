#include <functional>
#include <memory>
#include <thread>

#include "interfaces/action/plan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp" // We are dealing with components and not only simple nodes!

//#include "custom_action_cpp/visibility_control.h"

namespace plan_action
{
class PlannerActionServer : public rclcpp::Node
{
public:
  // Alias fot type to simplify the legibility of the code, c++ could be very verbose with so many template
  using Plan = interfaces::action::Plan;
  using GoalHandlePlan = rclcpp_action::ServerGoalHandle<Plan>;

  //GROUP_24_ASSIGNMENT_2_PUBLIC
  // ----- CONSTRUCTOR -----
  explicit PlannerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("plan_action_server", options)
  {
    using namespace std::placeholders;
    
    // Callback with lambda function for handling goals, it simply accept all goals
    auto handle_goal = [this]( // args of the lambda, // unique identifier of the goal, // pointer to .action message
      const rclcpp_action::GoalUUID & uuid, 
      std::shared_ptr<const Plan::Goal> goal) 
    { 
      RCLCPP_INFO(this->get_logger(), "Received goal request with target_ee_pose and type of movement  " ); //goal->order

      (void)uuid;// typical trick to suppress warining fo an unused variable
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // accept the goal and call the second callback handle_accepted()
    };

    // Callback with lambda function for dealing with cancellation, this implementation just tells the client that it accepted the cancellation
    auto handle_cancel = [this](
      
      const std::shared_ptr<GoalHandlePlan> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    // Callback with lambda function for accepting a new goal and starts processing it
    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandlePlan> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);}; // Launch of the execute() function
      std::thread{execute_in_thread}.detach();
    };

    // Constructor instantiate an ActionServer, in this object Server is implemeted the mechanism of binding,
    // so when a message is sent the various callbacks are called
    // In general an action server requires 6 things: 
    this->action_server_ = rclcpp_action::create_server<Plan>( // templated action type name
      this, // A ROS 2 node to add the action to
      "plan", // The action name
      handle_goal, // A callback function for handling goals
      handle_cancel,// A callback function for handling cancellation
      handle_accepted); // A callback function for handling goal accept
  }



private:
// ----- DATA MEMEBERS -----
  rclcpp_action::Server<Plan>::SharedPtr action_server_;


// ----- MEMBER FUNCTIONS -----

/* 
 Remmeber how is build the .action file: 
 Goal - Request - Feedback
*/
  // Function that process the goal required by the Client and response to it
  void execute(const std::shared_ptr<GoalHandlePlan> goal_handle) {
    
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(1); // 1 Hz freqenzy

    // Recover goal send by the Client
    const auto goal = goal_handle->get_goal(); 

//    // Inizialization of the Feedback and Result as shared pointer
    auto feedback = std::make_shared<Plan::Feedback>(); // Feedback
    auto & curr_pose = feedback->current_ee_pose; // Creare an Alias 
//    sequence.push_back(0);
//    sequence.push_back(1);
//    
    auto result = std::make_shared<Plan::Result>(); // Result

//    // Loop that iterates with 1 Hz frequency
//    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
//      // Check if there is a cancel request
//      // verify an internal status of the Action server, so if simultaneosly the Client has sent
//      // a notification of canceling
//      if (goal_handle->is_canceling()) {
//        result->sequence = sequence;
//        goal_handle->canceled(result);
//        RCLCPP_INFO(this->get_logger(), "Goal canceled");
//        return;
//      }
//      // Update sequence using the Plan recurency
//      sequence.push_back(sequence[i] + sequence[i - 1]);
//      // Publish feedback
//      goal_handle->publish_feedback(feedback);
//      RCLCPP_INFO(this->get_logger(), "Publish feedback");
//
//      loop_rate.sleep(); // sleep in order to mantain 1 Hz frequency
//    }
//
//    // Check if goal is done
//    if (rclcpp::ok()) { // Check if the node is still operative
//      // notice this line, at left sequence is the field of result described in .action, at right sequence 
//      // is the alias to the field partial_sequence of feedback. Indeed at the end the partial sequence correspond with the complete one
//      result->final_ee_pose = curr_pose; 
//      goal_handle->succeed(result); // Set goal SUCCEEDED and sent result to Client
//      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
//    }
  };

};  // class PlanActionServer

}  // namespace custom_action_cpp

// This macro is used when we deal with not simple nodes but for example with actions
// and allow this client to work 
int main(int argc, char ** argv)
{
  // Inizializza ROS 2
  rclcpp::init(argc, argv);

  // Crea il nodo (assicurati di usare il namespace corretto se ne hai uno)
  auto node = std::make_shared<plan_action::PlannerActionServer>();

  // Mantiene il nodo attivo (processa le callback)
  rclcpp::spin(node);

  // Chiude ROS 2 quando il nodo viene terminato (Ctrl+C)
  rclcpp::shutdown();

  return 0;
}
















/*
L'ultima riga del tuo file, al di fuori della classe e del namespace, è la chiave per far funzionare questo nodo come Componente ROS 2.

Ruolo e Importanza

- Registrazione (Registration): Questa macro inietta del codice C++ (spesso un boilerplate) che crea la funzione 
  di esportazione dinamica richiesta dal framework dei componenti ROS 2.

- Caricamento Dinamico: Quando avvii il component_container (o launch file che lo usa), il container cerca
  le librerie (.so su Linux) nel pacchetto e usa questa funzione registrata per sapere come creare un'istanza 
  della classe FibonacciActionServer all'interno del suo processo.

- Action Component: Poiché hai strutturato il tuo Action Server per accettare rclcpp::NodeOptions nel costruttore,
  questa macro lo registra come un nodo pronto per essere caricato dinamicamente, rendendolo un vero Action Component.

In sintesi, questa riga è il "timbro" che dice a ROS 2: "Questa libreria contiene un nodo chiamato
FibonacciActionServer e questo è il modo per avviarlo in un container!"
*/