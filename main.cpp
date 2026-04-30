#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iomanip>

using namespace std;

// Constants
const double G = 9.81;
const double PI = 3.14159265358979323846;

// --- Data Structures ---

struct Material {
    string name;
    double yieldStrength; // MPa
    double density;       // g/cm^3
};

struct Motor {
    string name;
    double torque;   // Nm
    double speed;    // RPM
    double mass;     // kg
    double diameter; // mm
    double width;    // mm
};

struct Gearbox {
    string name;
    double ratio;
    double efficiency;
    double mass;     // kg
    double diameter; // mm
    double width;    // mm
};

vector<Motor> motors = {
      // {"name",torque in N m, speedRpm, mass_kg, diameter mm , width mm}
    {"IDX 70 S, brushless, 600 W",  1.400, 5000,  1.595,  70, 125.5},
    {"IDX 70 M, brushless, 800 W",  2.480, 3670,    2.295,  70, 160.5},
    {"IDX 70 L, brushless, 900 W",  3.710, 2430,    2.995,  70, 195.5},
    {"IDX 56 L, brushless, 400 W",  1.040, 3430,    1.196,  56, 160},
    {"IDX 56 M, brushless, 350 W",  0.645, 5740,    0.815,  56, 130}



};

vector<Gearbox> gearboxes = {
       // {"name", efficiency, gearRatio, mass_kg, diameter mm , width mm}
    {"Planetary Gearhead GP 16 A, Metal Version, Sleeve Bearing", 0.85, 4.4, 0.020, 16, 18.1},
    {"Planetary Gearhead GP 22 A, Metal Version", 0.59, 84, 0.068, 22, 40},
    {"Planetary Gearhead GP 22 C", 0.70, 14, 0.055, 22, 35.35},
    {"Planetary Gearhead GP 32 A, Metal Version", 0.60, 246, 0.226, 32, 54.85},
    {"Planetary Gearhead GP 42 C, 3 - 15 Nm, Ceramic Version", 0.72, 43, 0.460, 42, 73}


};

vector<Material> materials = {
    // Name, Yield(MPa), Density(g/cm^3)
    {"Cast iron",       130, 7.30},
    {"Copper Nickel",   130, 8.94},
    {"Brass",           200, 8.73},
    {"Aluminum",        241, 2.70},
    {"Steel",           247, 7.58},
    {"Acrylic",         72,  1.16},
    {"Copper",          70,  8.92},
    {"Stainless steel", 275, 7.86},
    {"Tungsten",        941, 19.25}
};
// --- Stress Analysis & Optimization ---

void optimizeLink(Material mat, int type, double& dim1_mm, double& dim2_mm, double L_mm, double mp_kg, double alpha) {
    double rho_kg_m3 = mat.density * 1000.0;
    double sigma_yield_pa = mat.yieldStrength * 1e6;
    double L_m = L_mm / 1000.0;

    double dim1_m = dim1_mm / 1000.0;
    double dim2_m = dim2_mm / 1000.0;

    double sigma_pa = 0;
    double ml_kg, I_m4, M_nm;

    while (true) {
        // 1. Calculate Mass
        if (type == 1) // Rectangle
            ml_kg = rho_kg_m3 * (dim1_m * dim2_m * L_m);
        else           // Circle
            ml_kg = rho_kg_m3 * (PI * pow(dim1_m, 2) * L_m);

        // 2. Calculate Moment (Static + Inertial)
        double weightEffect = (ml_kg * G * (L_m / 2.0)) + (mp_kg * G * L_m);
        double inertialEffect = (ml_kg * pow(L_m / 2.0, 2) * alpha) + (mp_kg * pow(L_m, 2) * alpha);
        M_nm = weightEffect + inertialEffect;

        // 3. Moment of Inertia & Stress
        if (type == 1) { // Rectangle
            I_m4 = (dim1_m * pow(dim2_m, 3)) / 12.0;
            sigma_pa = (M_nm * (dim2_m / 2.0)) / I_m4;
        } else {         // Circle
            I_m4 = (PI * pow(dim1_m, 4)) / 4.0;
            sigma_pa = (M_nm * dim1_m) / I_m4;
        }

        // 4. Optimization Logic
        if (sigma_pa > sigma_yield_pa) {
            dim1_m *= 1.01;
            if (type == 1) dim2_m *= 1.01;
        } else if (sigma_pa < (sigma_yield_pa * 0.95)) { // 5% margin
            dim1_m *= 0.99;
            if (type == 1) dim2_m *= 0.99;
        } else {
            break;
        }
    }

    dim1_mm = dim1_m * 1000.0;
    dim2_mm = dim2_m * 1000.0;

    cout << "\n--- Optimized Link Results ---" << endl;
    cout << "Final Dimension(s): " << dim1_mm << (type == 1 ? " x " + to_string(dim2_mm) : "") << " mm" << endl;
    cout << "Final Mass: " << fixed << setprecision(3) << ml_kg << " kg" << endl;
    cout << "Final Stress: " << sigma_pa / 1e6 << " MPa" << endl;
}

void selectActuation(double ml_kg, double L_mm, double mp_kg, double alpha, const vector<Motor>& motors, const vector<Gearbox>& gearboxes) {
    double L_m = L_mm / 1000.0;
    double T_req = (ml_kg * G * (L_m / 2.0)) + (mp_kg * G * L_m) + (ml_kg * pow(L_m / 2.0, 2) * alpha) + (mp_kg * pow(L_m, 2) * alpha);

    double bestCost = 1e18;
    string bestCombo = "None";

    for (const auto& m : motors) {
        for (const auto& g : gearboxes) {
            double T_out = m.torque * g.ratio * g.efficiency;

            if (T_out >= T_req) {
                double totalMass = m.mass + g.mass;
                double maxDiam = max(m.diameter, g.diameter);
                double totalWidth = m.width + g.width;
                double cost = totalMass + (maxDiam / 100.0) + (totalWidth / 100.0);

                if (cost < bestCost) {
                    bestCost = cost;
                    bestCombo = m.name + " + " + g.name;
                }
            }
        }
    }

    cout << "\n--- Actuation Selection ---" << endl;
    cout << "Required Torque: " << T_req << " Nm" << endl;
    cout << "Best Combination: " << bestCombo << " (Cost Score: " << bestCost << ")" << endl;
}

int main() {

    int shapeChoice, matChoice;
    double L, mp, alpha, d1, d2 = 0;
    Material selectedMaterial;

    cout <<"Robot link optimizer made by team 7"<<endl;
    cout <<"-----------------------------------"<<endl;
    // 1. Shape Selection
    cout << "Select Link's cross section Shape (0 for Circle, 1 for Rectangle): "; cin >> shapeChoice;

    // 2. Material Selection
    cout << "Materials available:\n";
    for (int i = 0; i < materials.size(); i++) {
        cout << i << ": " << materials[i].name << " (Yield: " << materials[i].yieldStrength << "MPa)\n";
    }
    cout << materials.size() << ": Define Custom Material\n";
    cout << "Choice: "; cin >> matChoice;

    if (matChoice == materials.size()) {
        cout << "Enter Material Name: "; cin >> selectedMaterial.name;
        cout << "Enter Yield Strength (MPa): "; cin >> selectedMaterial.yieldStrength;
        cout << "Enter Density (g/cm^3): "; cin >> selectedMaterial.density;
    } else {
        selectedMaterial = materials[matChoice];
    }

    // 3. Dimensional Inputs
    cout << "Link Length (mm): "; cin >> L;
    cout << "Payload Mass (kg): "; cin >> mp;
    cout << "Max Acceleration (rad/s^2): "; cin >> alpha;

    if (shapeChoice) {
        cout << "Initial Width (mm): "; cin >> d1;
        cout << "Initial Height (mm): "; cin >> d2;
    } else {
        cout << "Initial Radius (mm): "; cin >> d1;
    }

    // Execution
    optimizeLink(selectedMaterial, shapeChoice, d1, d2, L, mp, alpha);

    // Final link mass calculation for motor selection
    double rho = selectedMaterial.density * 1000.0;
    double L_m = L / 1000.0;
    double d1_m = d1 / 1000.0;
    double d2_m = d2 / 1000.0;
    double ml = (shapeChoice == 1) ? rho*(d1_m*d2_m*L_m) : rho*(PI*pow(d1_m,2)*L_m);

    selectActuation(ml, L, mp, alpha, motors, gearboxes);

    cin.get();

    return 0;
}
