import copy

import serial
import numpy as np
import open3d as o3d
import quaternion


def update_mesh_rotation(mesh, vis, quat, initial_mesh):
    """
    Update the mesh rotation based on a given quaternion relative to its initial position.

    Parameters:
    - mesh: The mesh to rotate.
    - vis: The Open3D visualizer object.
    - quat: The quaternion representing the rotation.
    - initial_mesh: The initial state of the mesh.
    """
    # Convert the quaternion to a rotation matrix
    R = quaternion.as_rotation_matrix(quat)

    # Reset the mesh to its initial state
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(initial_mesh.vertices))
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(initial_mesh.triangles))
    mesh.compute_vertex_normals()

    # Rotate the mesh around its center of mass
    mesh.rotate(R, center=mesh.get_center())

    # Update the mesh in the visualizer
    vis.update_geometry(mesh)
    vis.poll_events()
    vis.update_renderer()


# Function to parse the incoming data
def parse_quaternion_data(data):
    try:
        # Strip the comment symbols and split by comma
        if data.startswith('/*'):
            parts = data.replace('/*', '').replace('*/', '').split(',')
            # Convert string data to float
            accuracy, w, x, y, z = map(float, parts)
            return quaternion.quaternion(w, x, y, z)
        else:
            return None
    except ValueError as e:
        print(f"Error parsing data: {e}")
        return None


if __name__ == '__main__':
    print(o3d.__version__)

    ser = serial.Serial('/dev/cu.wchusbserial54CE0362981', 115200, timeout=1)  # Update COM port and baud rate as needed

    # Load mesh, together with setting the flag for post-processing to True, so the texture and material will be loaded
    mesh_path = 'paper_plane.stl'
    mesh = o3d.io.read_triangle_mesh(mesh_path, True)

    # knot_mesh = o3d.data.KnotMesh()
    # mesh = o3d.io.read_triangle_mesh(knot_mesh.path)
    print(mesh)
    print('Vertices:')
    print(np.asarray(mesh.vertices))
    print('Triangles:')
    print(np.asarray(mesh.triangles))
    print(mesh.get_center())
    mesh.compute_vertex_normals()
    center_of_mass = mesh.get_center()

    # Define the rotation angle in radians (90 degrees)
    angle = np.pi / 2

    # Create a rotation matrix for a 90 degree rotation around the X-axis
    rotation_matrix = mesh.get_rotation_matrix_from_xyz((-angle, 0, 0))

    # Rotate the mesh
    mesh.rotate(rotation_matrix, center=center_of_mass)
    mesh_initial = copy.deepcopy(mesh)
    # Save the rotated mesh, if needed
    # o3d.io.write_triangle_mesh("path_to_rotated_stl_file.stl", mesh)

    # Create a small sphere to represent the center of mass
    center_of_mass_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=10.0)
    center_of_mass_sphere.translate(center_of_mass)
    center_of_mass_sphere.compute_vertex_normals()

    aabb = mesh.get_axis_aligned_bounding_box()

    # Create a coordinate frame at the center of mass
    #
    # - X-axis: Red
    # - Y-axis: Green
    # - Z-axis: Blue
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100.1, origin=center_of_mass)
    rotation_matrix = coordinate_frame.get_rotation_matrix_from_xyz((-angle, 0, 0))
    # coordinate_frame.rotate(rotation_matrix, center=center_of_mass)

    # Create a ground plane (large enough to act as a horizon)
    ground_plane = o3d.geometry.TriangleMesh.create_box(width=100, height=0.01, depth=100)
    ground_plane.translate([aabb.get_center()[0], aabb.get_min_bound()[1] - 0.01, aabb.get_center()[2]])
    ground_plane.paint_uniform_color([0.5, 0.5, 0.5])  # Give the ground plane a neutral color

    # Create a visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # Set background color to simulate sky
    vis.get_render_option().background_color = np.array([0.6, 0.8, 0.9])  # Light blue for sky

    # Add the original mesh
    vis.add_geometry(mesh)

    # Add the bounding box. The bounding box is colored by default.
    # vis.add_geometry(aabb)

    # Add the center of mass sphere
    vis.add_geometry(center_of_mass_sphere)

    # Add the coordinate frame
    vis.add_geometry(coordinate_frame)

    # Run the visualizer

    try:
        while True:
            # Your code to get the quaternion 'quat' from sensor data or other source
            # quat = np.quaternion(0.5, 0, 0, 0.)
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                quat = parse_quaternion_data(line)
                print(quat)
                # quat = quaternion.quaternion(0.248797, -0.282392, -0.609913, 0.697392)

                # Call the function to update the mesh rotation
                if quat:
                    print(quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2)
                    update_mesh_rotation(mesh, vis, quat, mesh_initial)

                # Other code, if necessary
                # ...

    except KeyboardInterrupt:
        # Cleanup code if the loop is interrupted
        print("Loop interrupted by user.")
    finally:
        vis.destroy_window()
    # vis.run() # This function will block the current thread until the window is closed.
    # vis.destroy_window()
