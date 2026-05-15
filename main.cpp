#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace std;

// Global Physical and Mathematical Constants
const double gravity = 9.81;                  // Acceleration due to gravity (m/s^2)
const double PI = 3.1415;
const double Rpm_to_rs = 0.1047;   // Precise value of PI for geometric calculations

// Class to store Material properties for stress and mass analysis
class Material {
public:
    string name;            // Name of the material (e.g., Aluminum, Steel)
    double yield_Strength;  // Yield strength in MegaPascals (MPa)
    double density;         // Material density in kg/m^3
};

// Class to store Motor specifications from manufacturer datasheets
class Motor {
public:
    string name;            // Motor model name
    double torque;          // Nominal continuous torque (Nm)
    double speed;           // No-load speed converted to rad/s
    double mass;            // Motor mass (kg)
    double diameter;        // Outer diameter of the motor (mm)
    double width;           // Axial length/width of the motor (mm)
};

// Class to store Gearbox specifications from manufacturer datasheets
class Gearbox {
public:
    string name;            // Gearbox model name
    double gear_ratio;      // Mechanical reduction ratio (G:1)
    double efficiency;      // Mechanical efficiency as a decimal factor (0.0 to 1.0)
    double mass;            // Gearbox mass (kg)
    double diameter;        // Outer diameter of the gearbox housing (mm)
    double width;           // Axial length of the gearbox (mm)
};

// Struct/Class to store geometric data for a Rectangular link cross-section
class Rectangle {
public:
    double width, length, height;
};

// Struct/Class to store geometric data for a Circular link cross-section
class Circle {
public:
    double length, radius;
};

// Database of available Maxon DC Motors
vector<Motor> motors = {
    {"ECX FLAT 32 L (100W, 48V)", 0.115, 13800.0 , 0.071, 32.0, 21.1},
    {"ECX FLAT 42 M (150W, 24V)", 0.285, 8050.0 , 0.127, 42.0, 17.2},
    {"ECX FLAT 42 M (150W, 48V)", 0.292, 7560.0 , 0.127, 42.0, 17.2},
    {"ECX FLAT 42 S (100W, 24V)", 0.171, 9120.0 , 0.106, 42.0, 12.2},
    {"ECX FLAT 42 S (100W, 48V)", 0.173, 8730.0 , 0.106, 42.0, 12.2},
    {"ECX PRIME 16 M (25W, 12V)", 0.0142, 15600.0 , 0.043, 16.0, 41.0}
};

// Database of available Maxon Gearheads
vector<Gearbox> gearboxes = {
    {"GB65 (Reduction 120:1)", 120.0, 0.60, 3.5, 65.0, 151.0},
    {"GB65 (Reduction 160:1)", 160.0, 0.55, 3.5, 65.0, 151.0},
    {"GP 16 A (Reduction 4.4:1)", 4.4, 0.90, 0.020, 16.0, 15.5},
    {"GP 16 A (Reduction 19:1)", 19.0, 0.81, 0.023, 16.0, 19.1},
    {"GP 16 A (Reduction 84:1)", 84.0, 0.73, 0.027, 16.0, 22.7},
    {"GP 16 A (Reduction 370:1)", 370.0, 0.65, 0.031, 16.0, 26.3},
    {"GP 16 A (Reduction 1621:1)", 1621.0, 0.59, 0.035, 16.0, 29.9}
};

// Database of Engineering Materials with true densities in kg/m^3
vector<Material> materials = {
    {"Cast iron", 130.0, 7300.0},
    {"Copper nickel", 130.0, 8940.0},
    {"Brass", 200.0, 8730.0},
    {"Aluminum", 241.0, 2700.0},
    {"Steel", 247.0, 7580.0},
    {"Acrylic", 72.0, 1160.0},
    {"Copper", 70.0, 8920.0},
    {"Stainless steel", 275.0, 7860.0},
    {"Tungsten", 941.0, 19250.0}
};


//Calculates geometric properties: Link Mass, Area Moment of Inertia (I), and neutral axis distance (y)
// shape: 1 for Rectangle, 2 for Circle

void calc_Shape_Properties(int shape, Material mat, Rectangle rect, Circle circ, double& Link_mass, double& Ineritia, double& y) {
    if (shape == 1) { // Cross-section: Rectangle
        Link_mass = mat.density * (rect.width * rect.height * rect.length);
        Ineritia = (rect.width * pow(rect.height, 3)) / 12.0; // I = (b * h^3) / 12
        y = rect.height / 2.0;                                // Max distance from neutral axis
    } else if (shape == 2) { // Cross-section: Solid Circle
        Link_mass = mat.density * (PI * pow(circ.radius, 2) * circ.length);
        Ineritia = (PI * pow(circ.radius, 4)) / 4.0;          // I = (pi * r^4) / 4
        y = circ.radius;                                      // Radius is the max distance
    }
}


  //Calculates total dynamic bending moment acting on the link joint
  //Combines Static Weight Effect and Inertial/Dynamic Effects

double calcMoment(double Link_mass, double payload_mass, double Length, double alpha_max) {
    // Torque caused by gravity acting on link center of mass and payload
    double weightEffect = (Link_mass * gravity * (Length / 2.0)) + (payload_mass * gravity * Length);

    // Torque caused by angular acceleration (Inertial resistance)
    double inertialEffect = (Link_mass * pow(Length / 2.0, 2) * alpha_max) + (payload_mass * pow(Length, 2) * alpha_max);

    return weightEffect + inertialEffect;
}


// Computes maximum bending stress experienced by the link using the flexure formula: sigma = (M * y) / I

double calc_max_Stress(double Moment, double y, double Ineritia) {
    return (Moment * y) / Ineritia;
}


 // Iterative Optimization Loop: Updates link dimensions until bending stress safely matches the material yield strength

void optimizeLink(int shape, Material mat, Rectangle& rect, Circle& circ, double payload_mass, double alpha_max, double Length, double& final_Moment)
 {
    double yield_stress_Pa = mat.yield_Strength * pow(10, 6); // Convert MPa to Pascals (Pa)

    double current_stress = 0;

    double Link_mass, Ineritia, y;

    while (true) {
        // Compute updated geometric properties
        calc_Shape_Properties(shape, mat, rect, circ, Link_mass, Ineritia, y);
        // Compute current total joint bending moment
        final_Moment = calcMoment(Link_mass, payload_mass, Length, alpha_max);
        // Calculate resulting bending stress
        current_stress = calc_max_Stress(final_Moment, y, Ineritia);

        // Optimization decision based on Factor of Safety and Yield Limits
        if (current_stress > yield_stress_Pa) {
            // Stress exceeds limit: increase dimensions by 1% to strengthen the structure
            if (shape == 1) {
                rect.width *= 1.01;
                rect.height *= 1.01;
            } else if (shape == 2) {
                circ.radius *= 1.01;
            }
        } else if (current_stress < (yield_stress_Pa * 0.98)) {
            // Stress is too low (Over-designed): reduce dimensions by 1% to minimize mass
            if (shape == 1)
                {
                rect.width *= 0.99;

                rect.height *= 0.99;

            } else if (shape == 2)
            {
                circ.radius *= 0.99;
            }
        } else {
            // Target convergence reached safely (Optimal design)
            break;
        }
    }
}

// Computes the actual output torque delivered after gearhead reduction and efficiency losses
double calc_Output_Torque(Gearbox gear, Motor motor)
 {
    return motor.torque * gear.gear_ratio * gear.efficiency;
}

// Computes reduced output angular speed at the gearhead output shaft
double calc_Output_Speed(Gearbox gear, Motor motor)
 {
    return (motor.speed / gear.gear_ratio)*Rpm_to_rs;
}


 // Multi-objective Cost Function evaluating physical size, weight, and footprint of the actuator assembly

double calc_Cost(Gearbox gear, Motor motor)
{
    double total_mass = gear.mass + motor.mass;

    double total_width = gear.width + motor.width;

    double max_diameter = (gear.diameter > motor.diameter) ? gear.diameter : motor.diameter;

    // Weighted optimization score (Lower score implies a more compact, lightweight powertrain)
    return total_mass + (max_diameter / 100.0) + (total_width / 100.0);
}

// Calculates minimum required operating velocity based on system kinematic constraints
double calc_w_required(double alpha_max, double length)
{
    return alpha_max / length;
}


 //Iterates through motor-gearbox combinations to select the optimal drive system meeting performance specs at minimum cost

void select_Optimal_Drive(double Torque_required, double w_required_val, const vector<Motor>& motors, const vector<Gearbox>& gearboxes, int& best_motor, int& best_gearbox, double& min_cost) {
    min_cost = pow(10, 9); // Initialize with a massive cost bound

    best_motor = 100;      // Sentinels indicating "Not Found"

    best_gearbox = 100;


    // Nested loop search across the entire drive matrix
    for (size_t i = 0; i < motors.size(); i++)
        {
        for (size_t j = 0; j < gearboxes.size(); j++)
         {
            double T_out = calc_Output_Torque(gearboxes[j], motors[i]);

            double w_out = calc_Output_Speed(gearboxes[j], motors[i]);

            // Filter out combinations that do not meet performance thresholds
            if (T_out >= Torque_required && w_out >= w_required_val)
                {
                double current_cost = calc_Cost(gearboxes[j], motors[i]);

                // Keep tracking the combination with the absolute lowest cost score
                if (current_cost < min_cost)
                    {
                    min_cost = current_cost;

                    best_motor = i;

                    best_gearbox = j;
                }
            }
        }
    }
}

int main() {
    int shape;

    int material_choice;

    double Length, payload_mass, alpha_max;

    // Code::Blocks console style configuration (Yellow on Blue)
    system("color E1");

    Rectangle rect;
    Circle circ;

    cout << "----------------------------- Welcome to Robot link design optimizer ---------------------------" << endl;
    cout << "----------------------------------------------------------------------------------------------" << endl;

    // UI: Material Selection Menu
    cout << "Available material:" << endl;

    for (size_t i = 0; i < materials.size(); i++)
        {
        cout << i + 1 << " : " << materials[i].name << endl;
    }
    cout << "10 - your custom material" << endl;

    cout << "Select link Material by entering its Number: ";

    cin >> material_choice;

    // Handle user defined custom material input

    if (material_choice == 10)
        {
        Material custom_material;

        cout << "Creating custom material... Enter in order: name, yield strength (MPa), density (kg/m³):" << endl;

        cin >> custom_material.name >> custom_material.yield_Strength >> custom_material.density;

        materials.push_back(custom_material);
    }

    int actual_mat_idx = material_choice - 1; // Map 1-based UI choice to 0-based vector index

    // Display chosen material metrics
    cout << "\nChosen material: " << materials[actual_mat_idx].name << "\n";

    cout << "Yield strength: " << materials[actual_mat_idx].yield_Strength << " MPa\n";

    cout << "Density: " << materials[actual_mat_idx].density << " kg/m³\n";


    cout << "-----------------------------------------------------------------" << endl;

    // UI: Kinematics and Geometry inputs

    cout << "Select cross section Shape (1-Rectangle, 2-Circle): ";

    cin >> shape;

    cout << "Link Length (m): ";

    cin >> Length;

    // Bind length to the specific shape instances to ensure analytical continuity
    rect.length = Length;

    circ.length = Length;

    cout << "Payload Mass (kg): ";

    cin >> payload_mass;

    cout << "Max radial Acceleration (rad/s^2): ";

    cin >> alpha_max;

    // Shape-specific initial boundary conditions
    if (shape == 1)
        {
        cout << "Initial width (m): ";

        cin >> rect.width;

        cout << "Initial Height (m): ";

        cin >> rect.height;

    } else if (shape == 2)
     {
        cout << "Initial Radius (m): ";

        cin >> circ.radius;

    } else
    {
        cout << "Error, Chose from cross sections that available." << endl;

        return 0;
    }

    double final_Moment = 0.0;

    int winner_motor = 100;

    int winner_gearbox = 100;

    double optimized_cost = 0.0;

    // Calculate required operational speed

    double w_req = calc_w_required(alpha_max, Length);

    // Execute structural optimization loop

    optimizeLink(shape, materials[actual_mat_idx], rect, circ, payload_mass, alpha_max, Length, final_Moment);

    // Execute drive optimization search matrix

    select_Optimal_Drive(final_Moment, w_req, motors, gearboxes, winner_motor, winner_gearbox, optimized_cost);

    // Print final engineering evaluation optimization report

    cout << "\n===========================================================" << endl;
    cout << "               FINAL OPTIMIZED DESIGN Tool              " << endl;
    cout << "===========================================================" << endl;

    cout << " OPTIMIZED LINK DIMENSIONS:" << endl;

    if (shape == 1)
        {
        cout << "     Final Link Width  : " << rect.width << " m" << endl;
        cout << "     Final Link Height : " << rect.height << " m" << endl;
    }
    else if (shape == 2)
        {
        cout << "    Final Link Radius : " << circ.radius << " m" << endl;
    }

    // Final mass computation after geometric convergence

    double rho = materials[actual_mat_idx].density;

    double final_mass = (shape == 1) ? rho * (rect.width * rect.height * Length) : rho * (PI * pow(circ.radius, 2) * Length);

    cout << "     Final Link Mass   : " << final_mass << " kg" << endl;

    cout << "-----------------------------------------------------------" << endl;
    cout << " OPTIMIZED ACTUATION SYSTEM:" << endl;
    cout << "    Required Torque to Lift System : " << final_Moment << " Nm" << endl;
    cout << "    Required Operating Speed       : " << w_req << " rad/s" << endl;
    cout << "-----------------------------------------------------------" << endl;

    // Print selection outcomes
    if (winner_motor != 100 && winner_gearbox != 100) {
        cout << "[SUCCESS] Best Match Component Combination Found:" << endl;
        cout << "    Selected Motor   : " << motors[winner_motor].name << endl;
        cout << "    Selected Gearbox : " << gearboxes[winner_gearbox].name << endl;
        cout << "    Total System Cost Score: " << optimized_cost << endl;
    } else {
        cout << "[WARNING] No suitable motor-gearbox combination found!" << endl;
        cout << "          The load is too heavy or speed is too high for your current database." << endl;
    }
    cout << "===========================================================" << endl;

    // Prevent sudden console termination on Windows execution environments
    cout << "\nPress Enter to exit the program...";
    cin.ignore();
    cin.get();

    return 0;
}
