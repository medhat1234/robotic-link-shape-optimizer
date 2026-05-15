#include <iostream>
#include <vector>
#include <string>
#include <cmath>


using namespace std;

// =====================================================================================
//                      1: Classes ,constants Definition
// =====================================================================================

//1 Basmalla, 2 Nour, 3 ahmed, 4 kimo, 5 hassouna , 6 mdht

const double gravity = 9.81;
const double PI = 3.1415;

class Material {
public:
    string name;
    double yield_Strength, density;
};

class Motor {
public:
    string name;
    double torque, speed, mass, diameter, width, cost;
};

class Gearbox {
public:
    string name;
    double gear_ratio, efficiency, mass, diameter, width, cost;
};

class Rectangle {
public:
    double width, length, height;
};

class Circle {
public:
    double length, radius;
};

// =====================================================================================
//                      2: Database Vectors
// =====================================================================================
vector<Motor> motors = {
 // {"name",torque in N m, speedRpm, mass_kg, diameter mm , width mm}
    {"IDX 70 S, brushless, 600 W",  1.400, 5000,  1.595,  70, 125.5},
    {"IDX 70 S, brushless, 600 W",  1.400, 5000,    1.595,  70, 125.5},
    {"IDX 70 M, brushless, 800 W",  2.480, 3670,    2.295,  70, 160.5},
    {"IDX 70 L, brushless, 900 W",  3.710, 2430,    2.995,  70, 195.5},
    {"IDX 56 L, brushless, 400 W",  1.040, 3430,    1.196,  56, 160},
};

vector<Gearbox> gearboxes = {
//  {"name", efficiency, gearRatio, mass_kg, diameter mm , width mm}
    {"Planetary Gearhead GP 16 A, Metal Version, Sleeve Bearing", 0.85, 4.4, 0.020, 16, 18.1},
    {"Planetary Gearhead GP 22 A, Metal Version", 0.59, 84, 0.068, 22, 40},
    {"Planetary Gearhead GP 22 C", 0.70, 14, 0.055, 22, 35.35},
    {"Planetary Gearhead GP 32 A, Metal Version", 0.60, 246, 0.226, 32, 54.85},
    {"Planetary Gearhead GP 42 C, 3 - 15 Nm, Ceramic Version", 0.72, 43, 0.460, 42, 73}
};

vector<Material> materials = {
    {"Cast iron", 130, 7.3},
    {"Aluminum", 241, 2.7},
    {"Steel", 247, 7.58},
    {"Stainless steel", 275, 7.86},
    {"Acrylic", 72, 1.16},
    {"Tungsten", 941, 19.25}
};

// =====================================================================================
//                       3: Physics Functions
// =====================================================================================

void calc_Shape_Properties(int shape, Material mat, Rectangle rect, Circle circ, double& mass_Link, double& Inertia, double& y) {
    double density_kg = mat.density * 1000.0;
    if (shape == 1) {
        mass_Link = density_kg * (rect.width * rect.height * rect.length);
        Inertia = (rect.width * pow(rect.height, 3)) / 12.0;
        y = rect.height / 2.0;
    } else {
        mass_Link = density_kg * (PI * pow(circ.radius, 2) * circ.length);
        Inertia = (PI * pow(circ.radius, 4)) / 4.0;
        y = circ.radius;
    }
}

double calc_Required_Torque(double mass_Link, double payload_mass, double Length, double alpha_max) {
    double weightEffect = (mass_Link * gravity * (Length / 2.0)) + (payload_mass * gravity * Length);
    double inertialEffect = (mass_Link * pow(Length / 2.0, 2) * alpha_max) + (payload_mass * pow(Length, 2) * alpha_max);
    return weightEffect + inertialEffect;
}

double calc_max_Stress(double Torque, double y, double Inertia) {
    return (Torque * y) / Inertia;
}

// =====================================================================================
//                       4: Optimize link
// =====================================================================================

void optimizeLink(int shape, Material mat, Rectangle& rect, Circle& circ, double payload_mass, double alpha_max, double& final_Torque) {
    double yield_stress_Pa = mat.yield_Strength * 1e6;
    double current_stress = 0;
    double mass_L, Inert, y_val;
    double L = (shape == 1) ? rect.length : circ.length;

    while (true) {
        calc_Shape_Properties(shape, mat, rect, circ, mass_L, Inert, y_val);
        final_Torque = calc_Required_Torque(mass_L, payload_mass, L, alpha_max);
        current_stress = calc_max_Stress(final_Torque, y_val, Inert);

        if (current_stress > yield_stress_Pa) {
            if (shape == 1) { rect.width *= 1.01; rect.height *= 1.01; }
            else { circ.radius *= 1.01; }
        } else if (current_stress < (yield_stress_Pa * 0.95)) {
            if (shape == 1) { rect.width *= 0.99; rect.height *= 0.99; }
            else { circ.radius *= 0.99; }
        } else { break; }
    }
}

// =====================================================================================
//                       5: Actuation Selection Functions
// =====================================================================================

double calc_Output_Torque(Gearbox gear, Motor motor) {
    return motor.torque * gear.gear_ratio * gear.efficiency;
}

double calc_Output_Speed(Gearbox gear, Motor motor) {
    return motor.speed / gear.gear_ratio;
}

double calc_Cost(Gearbox gear, Motor motor) {
    double total_mass = gear.mass + motor.mass;
    double max_diameter = max(gear.diameter, motor.diameter);
    double total_width = gear.width + motor.width;
    return total_mass + (max_diameter / 100.0) + (total_width / 100.0);
}

void select_Optimal_Drive(double Torque_required, double w_required, const vector<Motor>& motors, const vector<Gearbox>& gearboxes) {
    double min_cost = 1e9;
    int best_motor = 100;
    int best_gearbox = 100;

    for (int i = 0; i < motors.size(); i++) {
        for (int j = 0; j < gearboxes.size(); j++) {
            double T_out = calc_Output_Torque(gearboxes[j], motors[i]);
            double w_out = calc_Output_Speed(gearboxes[j], motors[i]);

            if (T_out >= Torque_required && w_out >= w_required) {
                double current_cost = calc_Cost(gearboxes[j], motors[i]);
                if (current_cost < min_cost) {
                    min_cost = current_cost;
                    best_motor = i;
                    best_gearbox = j;
                }
            }
        }
    }

    cout << "\n================= Drive Selection Results =================" << endl;
    if (best_motor != 100) {
        cout << "Required Torque : " << Torque_required << " Nm" << endl;
        cout << "Required Speed  : " << w_required << " rad/s" << endl;
        cout << "---------------------------------------------------------" << endl;
        cout << "[+] Best Combination Found:" << endl;
        cout << "    Motor   : " << motors[best_motor].name << endl;
        cout << "    Gearbox : " << gearboxes[best_gearbox].name << endl;
        cout << "    Cost Score: " << min_cost << endl;
    } else {
        cout << "WARNING: No motor-gearbox combination satisfies the requirements!" << endl;
    }
    cout << "===========================================================" << endl;
}

// =====================================================================================
//                      6: Main function
// =====================================================================================


int main() {
    int shape, mat_choice;
    double L, mp, alpha, w_req;
    Rectangle rect; Circle circ;

    cout << "Available Materials:\n";
    for(int i=0; i<materials.size(); i++) {
        cout << i+1 << ": " << materials[i].name << endl;
    }
    // Add a new option for Custom Material
    int custom_option = materials.size() + 1;
    cout << custom_option << ": [Add Custom Material]" << endl;

    cout << "Select Material (1-" << custom_option << "): ";
    cin >> mat_choice;

    Material selectedMaterial;

    if (mat_choice == custom_option) {
        // Logic to add a new material on the fly
        string c_name;
        double c_yield, c_density;
        cout << "Enter Material Name: ";
        cin.ignore(); // Clear buffer
        getline(cin, c_name);
        cout << "Enter Yield Strength (MPa): ";
        cin >> c_yield;
        cout << "Enter Density (g/cm^3): ";
        cin >> c_density;

        selectedMaterial = {c_name, c_yield, c_density};
        // Optional: Add it to the vector if you want to save it for this session
        materials.push_back(selectedMaterial);
    } else if (mat_choice > 0 && mat_choice <= materials.size()) {
        selectedMaterial = materials[mat_choice - 1];
    } else {
        cout << "Invalid selection!" << endl;
        return 1;
    }

    // --- Rest of the inputs ---
    cout << "\nSelect Shape (1:Rectangle, 2:Circle): ";
    cin >> shape;
    cout << "Link Length (m): ";
    cin >> L;
    cout << "Payload Mass (kg): ";
    cin >> mp;
    cout << "Max Acceleration (rad/s^2): ";
    cin >> alpha;
    cout << "Required Speed (rad/s): ";
    cin >> w_req;

    if (shape == 1) {
        rect.length = L;
        cout << "Initial Width, Height (m): ";
        cin >> rect.width >> rect.height;
    } else {
        circ.length = L;
        cout << "Initial Radius (m): ";
        cin >> circ.radius;
    }

    // --- Logic Execution (Using selectedMaterial) ---
    double final_Torque = 0.0;

    // 1. Optimize Link & Get required Torque
    optimizeLink(shape, selectedMaterial, rect, circ, mp, alpha, final_Torque);

    // 2. Display Optimized Dimensions
    cout << "\n--- Optimized Link Results ---" << endl;
    cout << "Material Used: " << selectedMaterial.name << endl;
    if (shape == 1) cout << "Final Dimensions: " << rect.width << " x " << rect.height << " m" << endl;
    else cout << "Final Radius: " << circ.radius << " m" << endl;

    // 3. Recalculate Final Mass
    double rho = selectedMaterial.density * 1000.0;
    double mass_final = (shape == 1) ? rho*(rect.width*rect.height*L) : rho*(PI*pow(circ.radius,2)*L);
    cout << "Final Link Mass: " << mass_final << " kg" << endl;

    // 4. Select the Best Drive
    select_Optimal_Drive(final_Torque, w_req, motors, gearboxes);

    cout << "\nPress Enter to Exit...";
    cin.ignore(); cin.get();
    return 0;
}
