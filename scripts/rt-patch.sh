#!/bin/bash
#
# Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
#
# The script applies/reverts PREEMPT RT patches in the kernel source.
# - executed in "scripts"
# - usage:
#         rt-patch.sh apply-patches; #for applying
#         rt-patch.sh revert-patches; #for reverting

any_failure=0

path_defconfig=../arch/arm64/configs
l4t_defconfig=tegra_defconfig
auto_defconfig=tegra_gnu_linux_defconfig

apply_rt_patches()
{
	count=$(ls $path_defconfig/.tmp.tegra*defconfig 2>/dev/null| wc -l)
	if [ $count -gt 0 ]; then
		echo "The PREEMPT RT patches are already applied to the kernel!"
	else
		file_list=`find ../rt-patches -name \*.patch -type f | sort`
		for p in $file_list; do
			# set flag in case of failure and continue
			patch -s -d .. -p1 < $p || any_failure=1
		done

		for f in $path_defconfig/*; do
			defconfig=$f
			tmp_defconfig=${f//$path_defconfig\//$path_defconfig\/.tmp.}

			if [[ "$f" == "$path_defconfig/$l4t_defconfig" ||
			      "$f" == "$path_defconfig/$auto_defconfig" ]]; then
				if [ ! -e "$tmp_defconfig" ]; then
					#make temporary copy of defconfig file
					cp $defconfig $tmp_defconfig
					echo "Applying RT macro's to:"\
					"`(echo $defconfig | cut -d "/" -f5)`"
					./config --file $defconfig\
					    --enable PREEMPT_RT_FULL \
					    --disable DEBUG_PREEMPT \
					    --disable CPU_IDLE_TEGRA18X \
					    --disable CPU_FREQ_TIMES \
					    --disable CPU_FREQ_GOV_SCHEDUTIL \
					    --disable CPU_FREQ_GOV_INTERACTIVE \
						|| any_failure=1
					echo "PREEMPT RT patches successfully"\
								"applied!"
				fi
			fi
		done
	fi
}

revert_rt_patches()
{
	count=$(ls $path_defconfig/.tmp.tegra*defconfig 2>/dev/null| wc -l)
	if [ $count -gt 0 ]; then
		file_list=`find ../rt-patches -name \*.patch -type f | sort -r`
		for p in $file_list; do
			# set flag in case of failure and continue
			patch -s -R -d .. -p1 < $p || any_failure=1
		done
		for f in $path_defconfig/.tmp*; do
			defconfig=`echo $f | cut -d "." -f5`
			cp $f $path_defconfig/$defconfig
			rm -rf $f
		done
		echo "The PREEMPT RT patches have been successfully reverted!"
	else
		echo "The PREEMPT RT patches are not applied to the kernel!"
	fi
}

usage()
{
	echo Usages:
	echo 1. ${0} apply-patches : Apply RT patches
	echo 2. ${0} revert-patches : Revert RT patches
	any_failure=1
}

# script starts from here
dir_run_from=`dirname ${0}`
pushd $dir_run_from &>/dev/null

if [ "$1" == "apply-patches" ]; then
	apply_rt_patches
elif [ "$1" == "revert-patches" ]; then
	revert_rt_patches
else
	echo "Wrong argument"
	usage
fi

popd &>/dev/null

exit $any_failure
