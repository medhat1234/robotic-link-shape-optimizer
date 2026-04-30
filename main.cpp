#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

using namespace std;

// --- Physical Constants ---
const double GRAVITY_MM_S2 = 9810.0; // Gravity in mm/s^2
const double PI = 3.14159265358979323846;
// Converts g/cm^3 to g/mm^3 (1 cm^3 = 1000 mm^3)
const double DENSITY_CONVERSION = 0.001;

struct Material {
    string name;
    double yieldStrengthMPa;
    double densityG_CM3;
};

struct Motor { //dont use frameless
    string name;
    double torqueNmm;
    double efficiency;

    double speedRPM;
    double massGrams;
    double diameterMM;
    double widthMM;

};

struct Gearbox {
    string name;
    double torqueNmm;
    double efficiency;

    double gearRatio;
    double massGrams;
    double diameterMM;
    double widthMM;
};

// Global Data Collections
vector<Motor> motors = {
      // {"name",torque in N mm, efficiency, speedRpm, mass_grams, diameter mm , width mm}
    {"IDX 56 L, Ø56 mm, brushless, 400 W, with Hall sensors and Encoder EASY INT 1024CPT",  1040, 0.85,  2900,  1196,  56, 160},
    {"IDX 70 L, Ø70 mm, brushless, 900 W, with Hall sensors and Encoder EASY INT 1024CPT",  2840, 0.85,  1920,    82,  22, 195},


};

vector<Gearbox> gearboxes = {
       // {"name",torque in N mm, efficiency, gearRatio, mass_grams, diameter mm , width mm}
    {"Planetary Standard", 50000, 0.85, 50, 300, 40, 30}

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

// --- Part 1: Stress Analysis & Optimization ---

void optimizeLink(Material selectedMat, bool isRectangular, double& dim1, double& dim2,
                  double linkLengthMM, double payloadMassGrams, double angularAccel) {

    double yieldLimit = selectedMat.yieldStrengthMPa;
    // Convert g/cm^3 to g/mm^3 for internal math
    double internalDensity = selectedMat.densityG_CM3 * DENSITY_CONVERSION;

    double calculatedStress = 0;
    double linkMassGrams, bendingMoment;

    while (true) {
        // 1. Calculate Mass (Volume in mm^3 * Density in g/mm^3)
        if (isRectangular) {
            linkMassGrams = internalDensity * (dim1 * dim2 * linkLengthMM);
        } else {
            linkMassGrams = internalDensity * (PI * pow(dim1, 2) * linkLengthMM);
        }

        // 2. Calculate Bending Moment (N*mm)
        double weightEffect = (linkMassGrams * GRAVITY_MM_S2 * (linkLengthMM / 2.0)) +
                              (payloadMassGrams * GRAVITY_MM_S2 * linkLengthMM);

        double inertialEffect = (linkMassGrams * pow(linkLengthMM / 2.0, 2) * angularAccel) +
                                (payloadMassGrams * pow(linkLengthMM, 2) * angularAccel);

        bendingMoment = (weightEffect + inertialEffect) / 1000.0;

        // 3. Moment of Inertia & Stress (MPa)
        if (isRectangular) {
            double momentOfInertia = (dim1 * pow(dim2, 3)) / 12.0;
            calculatedStress = (bendingMoment * (dim2 / 2.0)) / momentOfInertia;
        } else {
            double momentOfInertia = (PI * pow(dim1, 4)) / 4.0;
            calculatedStress = (bendingMoment * dim1) / momentOfInertia;
        }

        // 4. Iterative Optimization
        if (calculatedStress > yieldLimit) {
            dim1 *= 1.01;
            if (isRectangular) dim2 *= 1.01;
        } else if (calculatedStress < (yieldLimit * 0.96)) {
            dim1 *= 0.99;
            if (isRectangular) dim2 *= 0.99;
        } else {
            break;
        }
    }

    //cout << fixed << setprecision(2);
    cout << "\n--- Optimized Link Results ---" << endl;
    cout << "Final Dimension(s): " << dim1 << (isRectangular ? " x " + to_string(dim2) : "") << " mm" << endl;
    cout << "Final Mass: " << linkMassGrams << " g" << endl;
    cout << "Final Stress: " << calculatedStress << " MPa" << endl;
}

// --- Part 2: Motor & Gearbox Selection ---

void selectActuation(double linkMassG, double lengthMM, double payloadMassG,
                     double angularAccel, vector<Motor>& motorList, vector<Gearbox>& gearboxList) {

    double torqueReq = ((linkMassG * GRAVITY_MM_S2 * (lengthMM / 2.0)) +
                        (payloadMassG * GRAVITY_MM_S2 * lengthMM) +
                        (linkMassG * pow(lengthMM / 2.0, 2) * angularAccel) +
                        (payloadMassG * pow(lengthMM, 2) * angularAccel)) / 1000.0;

    double bestCost = 1e18;
    string bestCombo = "None";

    for (const auto& m : motorList) {
        for (const auto& g : gearboxList) {
            double outputTorque = m.torqueNmm * g.gearRatio * g.efficiency*m.efficiency;
            if(outputTorque > g.torqueNmm) //capping the output torque to max gearbox can withstand
            {
                outputTorque = g.torqueNmm;
            }

            if (outputTorque >= torqueReq) {
                double totalMass = m.massGrams + g.massGrams;
                double maxDiameter = max(m.diameterMM, g.diameterMM);
                double totalWidth = m.widthMM + g.widthMM;

                double cost = totalMass + (maxDiameter * 10) + (totalWidth * 10);

                if (cost < bestCost) {
                    bestCost = cost;
                    bestCombo = m.name + " + " + g.name;
                }
            }
        }
    }

    cout << "\n--- Actuation Selection ---" << endl;
    cout << "Required Torque: " << torqueReq/1000 << " N.m" << endl;
    cout << "Best Combination: " << bestCombo << " (Cost score: " << bestCost << ")" << endl;
}

int main() {
    int shapeChoice;
    int materialIndex;
    double linkLength, payloadMass, accel, dim1, dim2 = 0;

    cout << "================ ROBOTIC LINK OPTIMIZER ================" << endl;

    for(int i = 0; i < materials.size(); i++) {
        cout << i << " : " << materials[i].name << endl;
    }
    cout << "9 : Define Custom Material" << endl;
    cout << "Choice: "; cin >> materialIndex;

    if(materialIndex == 9) {
        Material custom;
        cout << "Enter Name, Yield(MPa), Density(g/cm^3): ";
        cin >> custom.name >> custom.yieldStrengthMPa >> custom.densityG_CM3;
        materials.push_back(custom);
    }

    cout << "Select Shape (0: Circle, 1: Rectangle): "; cin >> shapeChoice;
    cout << "Link Length (mm): "; cin >> linkLength;
    cout << "Payload Mass (g): "; cin >> payloadMass;
    cout << "Angular Accel (rad/s^2): "; cin >> accel;
    cout << (shapeChoice ? "Initial Width (mm): " : "Initial Radius (mm): "); cin >> dim1;
    if (shapeChoice) { cout << "Initial Height (mm): "; cin >> dim2; }

    bool isRect = (shapeChoice == 1);
    optimizeLink(materials[materialIndex], isRect, dim1, dim2, linkLength, payloadMass, accel);

    double internalDensity = materials[materialIndex].densityG_CM3 * DENSITY_CONVERSION;
    double finalMass = isRect ? internalDensity*(dim1 * dim2 * linkLength) : internalDensity*(PI * pow(dim1, 2) * linkLength);

    selectActuation(finalMass, linkLength, payloadMass, accel, motors, gearboxes);

    cout << "\nPress Enter to exit...";
    cin.ignore();
    cin.get();
    return 0;
}
