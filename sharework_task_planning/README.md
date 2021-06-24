Launchers to run task_planner_interface with Sharework cell.

- hrc_fake.launch is intended to be used for simulations
- hrc_real.launch is intended to be used for real experiments

Both launchers runs an instance of robot_node to execution robot actions. The two launchers differ in the how they handle tasks of humans:

- hrc_fake.launch runs a second istance of robot_node, assuming a move group to simulate human's action is present
- hrc_real.launch runs human_node to receives feedbacks from actual human users

## Usage in simulation

Run an instance of mongod (path may vary depending on your mongo settings):
```
mongod --dbpath ~/data/db

```
Launch sharework_cembre cell:
```
roslaunch sharework_cembre_configurations fake.launch
```
Launch manipulation skill servers:
```
roslaunch sharework_cembre_skills skills.launch
```

(Optional) Launch GUI:
```
roslaunch task_planner_gui gui.launch
```

Launch task planning interface:
```
roslaunch sharework_task_planning hrc_fake.launch
```

## Parameters

- name="platinum" default="false" : if false, read a recipe from param recipe or recipe_folder
- name="hrc" default="false" : if true, runs an instance of human_node (or robot_node)
- name="recipe_path" value="path_to_folder" : used only if platinum=false; execute all the recipe in path_to_folder
- name="recipe" value="path_to_recipe" : used only if platinum = false and recipe_path is commented
- name="mongo_collection" default="string" : name of the mongo collection where to save results
