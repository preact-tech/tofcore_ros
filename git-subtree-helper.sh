#!/bin/bash
SOURCE_BRANCH="tofcore_ros_internal-source"
STAGING_BRANCH="tofcore_ros_internal-staging"

echo
echo "Fetch new changes from tofcore_ros_internal"
echo -----------------------------------------
git fetch tofcore_ros_internal develop

TOFCORE_ROS_LATEST_COMMIT=`git ls-remote tofcore_ros_internal | grep "refs/heads/develop" | awk '{ print $1}'`
echo
echo "tofcore_ros_internal latest commit: ${tofcore_ros_internal_LATEST_COMMIT}"
echo

# checkout source repo
git checkout -b ${SOURCE_BRANCH} tofcore_ros_internal/develop

ls
# create new staging branch from all the commits impacting "/my-chart" from source repo
git subtree split -P tofcore -b ${STAGING_BRANCH}

# checkout develop
git checkout main

# take commits in the staging branch and set "/helm-charts/my-chart" as the commit root
# after you run this script the first time, update the command below to:
# "git subtree merge" with the same parameters (previously was subtree add)  
#
echo
echo "Add in changes"
echo -----------------------------------------
git subtree merge -P ros_nodes ${STAGING_BRANCH} --message "Update tofcore from internal repo, commit: ${TOFCORE_ROS_LATEST_COMMIT}"



# clean up
echo
echo
git branch -D ${STAGING_BRANCH}
git branch -D ${SOURCE_BRANCH}
