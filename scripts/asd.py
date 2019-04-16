import rospkg
import pkg_resources
from rospy import logwarn
import rospy
import re
from pkg_resources import DistributionNotFound, VersionConflict

'''
dependencies = [
    #pip
    'symengine>=0.3.1.dev1',
    'numpy==1.15.4',
    'urdfdom-py==0.4.0',
    'rosunit==1.14.4',
    #apt
    'cython==0.26.1',
    #ROS
    'qpoases==3.1.0',
    'giskard_msgs<=0.1.0',
    'py_trees_ros>=0.5.14',
    'py_trees<=0.6.1',
    'test<=0.6.1'
]'''


def compare_version(version1, operator, version2):
    """
    compares two version numbers by means of the given operator
    :param version1: version number 1 e.g. 0.1.0
    :type version1: str
    :param operator: ==,<=,>=,<,>
    :type operator: str
    :param version2: version number 1 e.g. 3.2.0
    :type version2: str
    :return:
    """
    version1 = version1.split('.')
    version2 = version2.split('.')
    if operator == '==':
        if (len(version1) != len(version2)):
            return False
        for i in range(len(version1)):
            if version1[i] != version2[i]:
                return False
        return True
    elif operator == '<=':
        k = min(len(version1), len(version2))
        for i in range(k):
            if version1[i] > version2[i]:
                return True
            elif version1[i] < version2[i]:
                return False
        if len(version1) < len(version2):
            return False
        else:
            return True
    elif operator == '>=':
        k = min(len(version1), len(version2))
        for i in range(k):
            if version1[i] < version2[i]:
                return True
            elif version1[i] > version2[i]:
                return False
        if len(version1) > len(version2):
            return False
        else:
            return True
    elif operator == '<':
        k = min(len(version1), len(version2))
        for i in range(k):
            if version1[i] > version2[i]:
                return True
            elif version1[i] < version2[i]:
                return False
        if len(version1) < len(version2):
            return False
        else:
            return True
    elif operator == '>':
        k = min(len(version1), len(version2))
        for i in range(k):
            if version1[i] < version2[i]:
                return True
            elif version1[i] > version2[i]:
                return False
        if len(version1) > len(version2):
            return False
        else:
            return True
    else:
        return False


def rospkg_exists(name):
    """
       checks whether a ros package with the given name and version exists
       :param name: the name and version of the ros package in requirements format e.g. giskard_msgs<=0.1.0
       :type name: str
       :return: True if it exits else False
    """
    r = rospkg.RosPack()
    name = name.replace(' ', '')
    version_list = name.split(',')
    version_entry1 = re.split('(==|>=|<=|<|>)', version_list[0])
    package_name=version_entry1[0]
    try:
        m = r.get_manifest(package_name)
    except Exception as e:
        logwarn('package {name} not found'.format(name=name))
        return False
    if len(version_entry1) == 1:
        return True
    if not compare_version(version_entry1[2], version_entry1[1], m.version):
        logwarn('found ROS package {installed_name}=={installed_version} but {r} is required'.format(installed_name=package_name, installed_version=str(m.version), r=name))
        return False
    for entry in version_list[1:]:
        operator_and_version = re.split('(==|>=|<=|<|>)', entry)
        if not compare_version(operator_and_version[2], operator_and_version[1], m.version):
            logwarn('found ROS package {installed_name}=={installed_version} but {r} is required'.format(installed_name=package_name, installed_version=str(m.version), r=name))
            return False

    return True


def check_dependencies():
    """
    Checks whether the dependencies specified in the dependency.txt in the root folder of giskardpy are installed. If a
    dependecy is not installed a message is printed.
    """
    r = rospkg.RosPack()

    with open(r.get_path('giskardpy') + '/dependencies.txt') as f:
        dependencies = f.readlines()

    dependencies = [x.split('#')[0] for x in dependencies]
    dependencies = [x.strip() for x in dependencies]

    for d in dependencies:
        try:
            pkg_resources.require(d)
        except pkg_resources.DistributionNotFound as e:
            rospkg_exists(d)
        except pkg_resources.VersionConflict as e:
            logwarn('found {version_f} but version {version_r} is required'.format(version_r=str(e.req), version_f=str(e.dist)))

rospy.init_node('test')
check_dependencies()




















def rospkg_exits(name):
    """
    checks whether a ros package with the given name and version exists
    :param name: the name and version of the ros package in requirements format e.g. giskard_msgs<=0.1.0
    :return: True if it exits else False
    """
    r = rospkg.RosPack()
    try:
        l = name.split('=')
        if l[0].endswith('<'):
            m = r.get_manifest(l[0][:-1])
            version_m = m.version.split('.')
            version_d = l[len(l) - 1].split('.')
            version_d = l[len(l) - 1].split('.')
            k = max(len(version_m), len(version_d))
            for i in range(k):
                if version_m[i] < version_d[i]:
                    return True
                elif version_m[i] > version_d[i]:
                    logwarn("found ROS package " + l[0][:-1] + "==" + str(
                        m.version) + " but " + name + " is required")
                    return False
            return True
        elif l[0].endswith('>'):
            m = r.get_manifest(l[0][:-1])
            version_m = m.version.split('.')
            version_d = l[len(l) - 1].split('.')
            k = max(len(version_m), len(version_d))
            for i in range(k):
                if version_m[i] > version_d[i]:
                    return True
                elif version_m[i] < version_d[i]:
                    logwarn("found ROS package " + l[0][:-1] + "==" + str(
                        m.version) + " but " + name + " is required")
                    return False
            return True
        else:
            m = r.get_manifest(l[0])
            version_m = m.version.split('.')
            version_d = l[len(l)-1].split('.')
            if(len(version_m) != len(version_d)):
                logwarn("found ROS package " + l[0] + "==" + str(m.version) + " but " + name + " is required")
                return False
            for i in range(len(version_m)):
                if version_m[i] != version_d[i]:
                    logwarn("found ROS package " + l[0] + "==" + str(m.version) + " but " + name + " is required")
                    return False
            return True

    except Exception as e:
        logwarn(name + " not found")
        return False

#a = r.get_path('py_trees')
#b = r.get_manifest('cython')

#print(b.version)