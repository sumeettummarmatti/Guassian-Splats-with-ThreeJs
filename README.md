# Gaussian Splats with Three.js & Robot Simulation

This project is a sophisticated 3D visualization and interaction platform built with **Three.js** and **Gaussian Splatting**, utilizing the [Spark](https://github.com/sparkjsdev/spark) library for seamless integration. 
This project features a custom-built robot character and advanced mesh generation tools for collision detection, leveraging the **Convex Hull** algorithm.

## 🚀 Features

### 🤖 Custom Robot Character
A fully programmatic 3D robot built using Three.js primitives, designed for interaction within the scene.
- **Structure**:
  - **Head**: Features eyes and an antenna for a distinct look.
  - **Body**: Central unit with a backpack attachment.
  - **Arms**: Articulated with shoulder, elbow, and hand segments.
  - **Legs**: Articulated with hip, knee, and foot segments.
- **Integration**: The robot is integrated into the physics system for interaction with the environment.

### 🛠️ Mesh Generation & Physics
- **Algorithm**: We utilize the **Convex Hull** algorithm (via `ConvexGeometry`) to generate collision meshes.
- **Workflow**:
  - Users can select points in the Gaussian Splat cloud (e.g., using a Brush tool).
  - The system calculates the convex hull of these points to create a watertight 3D mesh.
  - This mesh is then used for physics collisions, allowing the robot and other objects to interact with the reconstructed environment.

### 🎨 3D Gaussian Splatting
- **Rendering**: High-fidelity real-time rendering of captured scenes.
- **Data Source**: The point cloud data (`work/export_last.ply`) was generated using the **NVIDIA 3DGUT** (3D Gaussian Unscented Transform) tool. You can find the tool and more details at [3DGRUT GitHub](https://github.com/nv-tlabs/3dgrut). This advanced tool allows for high-quality scene reconstruction which serves as the base for this project.

## 📦 Installation & Requirements

### Prerequisites
- **Node.js**: Required to run the development server and build tools. (v16 or higher recommended).
- **Python** (Optional): Only required if you plan to run external tools like 3DGUT.

### Setup

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd Guassian-Splats-with-ThreeJs
    ```

2.  **Install Dependencies**:
    ```bash
    npm install
    ```

3.  **Python Requirements** (If applicable for external scripts):
    If you have a `requirements.txt` for any Python tools used alongside this project, install them using:
    ```bash
    pip install -r requirements.txt
    ```
    *(Note: The core viewer is JavaScript-based and does not require Python to run).*

## 🏃‍♂️ Usage

### Development Server
To start the local development server with hot-reload:

```bash
npm run dev
```
Open your browser and navigate to the URL shown in the terminal (usually `http://localhost:8080`).

### Production Build
To build the project for production deployment:

```bash
npm run build
```

### Other Commands
- **Linting**: `npm run lint` - Check for code quality issues.
- **Formatting**: `npm run format` - Auto-format the code.
- **Preview**: `npm run start` - Preview the production build locally.

## 🎮 How to Use

1.  **Explore**: Use standard camera controls to navigate the 3D scene.
2.  **Interact**:
    - Use the **Brush Tool** to paint over areas of the Gaussian Splat.
    - The system will automatically generate a **Convex Hull** mesh around the painted area.
    - This creates a physical barrier that the robot can collide with, enabling dynamic interaction with the static scene.
