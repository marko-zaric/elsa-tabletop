# Object Database
The elsa_object_database package is a collection of registered object colors to help track feature changes of certain registered objects. It serves two main purposes:
- Labeling of objects in the perception step with a color label
- Controlled randomization of object colors for benchmarking purposes

## Storage
The object database is stored in the file *ObjectDatabase.csv*. It currently consists of 6 columns which are all used by various nodes in the **elsa_perception** package.

| Column index | Description | Type |
| :---:         | :----      | :---: |
| 0            | Object Label| String|
| 1            | HSV H-value | Float |
| 2            | Ambient RGBA R-value | Float |
| 3            | Ambient RGBA G-value | Float |
| 4            | Ambient RGBA B-value | Float |
| 5            | Ambient RGBA A-value | Float |

Some colors are predefined Gazebo Colors which in turn are stored like this:
| Column index | Description | Type |
| :---:         | :----      | :---: |
| 0            | Object Label| String|
| 1            | HSV H-value | Float |
| 2            | Gazebo Color string | String |
