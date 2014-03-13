/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <mach/cpufreq.h>

static struct cpus {
	bool throttling;
	int thermal_steps[5];
	uint32_t limited_max_freq;
	unsigned int max_freq;
	struct cpufreq_policy policy;
} cpu_stats = {
	.throttling = false,
	.thermal_steps = {729600, 1036800, 1267200, 1497600},
	.limited_max_freq = UINT_MAX,
};

unsigned int temp_threshold = 60;
module_param(temp_threshold, int, 0755);

static struct msm_thermal_data msm_thermal_info;

static struct workqueue_struct *wq;
static struct delayed_work check_temp_work;

unsigned short get_threshold(void)
{
	return temp_threshold;
}

static int  msm_thermal_cpufreq_callback(struct notifier_block *nfb,
		unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	if (policy->max == cpu_stats.limited_max_freq)
		return 0;
		
	cpufreq_verify_within_limits(policy, cpu_stats.policy.cpuinfo.min_freq,
		cpu_stats.limited_max_freq);

	return 0;
}

static struct notifier_block msm_thermal_cpufreq_notifier = {
	.notifier_call = msm_thermal_cpufreq_callback,
};

static void limit_cpu_freqs(uint32_t max_freq)
{
    int cpu;

	if (cpu_stats.limited_max_freq == max_freq)
		return;

	cpu_stats.limited_max_freq = max_freq;
    
	/* Update new limits */
	get_online_cpus();
	for_each_online_cpu(cpu)
	{
		cpufreq_update_policy(cpu);
		pr_info("%s: Setting cpu%d max frequency to %d\n",
				KBUILD_MODNAME, cpu, cpu_stats.limited_max_freq);
	}
	put_online_cpus();
}

static void check_temp(struct work_struct *work)
{
	struct tsens_device tsens_dev;
	long temp = 0;
	uint32_t freq;
    
	tsens_dev.sensor_num = msm_thermal_info.sensor_id;
	tsens_get_temp(&tsens_dev, &temp);

	cpufreq_get_policy(&cpu_stats.policy, 0);

	/* most of the time the device is not hot so reschedule early */
	if (likely(temp < temp_threshold))
	{
		if (unlikely(cpu_stats.throttling))
		{
			limit_cpu_freqs(cpu_stats.policy.cpuinfo.max_freq);
			cpu_stats.throttling = false;
		}

		goto reschedule;
	}

	if (temp >= (temp_threshold + 12))
		freq = cpu_stats.thermal_steps[0];
	else if (temp >= (temp_threshold + 9))
		freq = cpu_stats.thermal_steps[1];
	else if (temp >= (temp_threshold + 5))
		freq = cpu_stats.thermal_steps[2];
	else
		freq = cpu_stats.thermal_steps[3];

	limit_cpu_freqs(freq);

	cpu_stats.throttling = true;

reschedule:
	queue_delayed_work(wq, &check_temp_work, msecs_to_jiffies(250));
}

int __devinit msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;
    
	BUG_ON(!pdata);
	BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));
    
	wq = alloc_workqueue("msm_thermal_workqueue", WQ_UNBOUND, 0);
    
    if (!wq)
        return -ENOMEM;

	cpufreq_register_notifier(&msm_thermal_cpufreq_notifier,
			CPUFREQ_POLICY_NOTIFIER);
    
	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	queue_delayed_work(wq, &check_temp_work, HZ*30);
    
	return ret;
}

static int probe_cc(struct device_node *node, struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	uint32_t cpu_cnt = 0;
	int ret = 0;
	uint32_t cpu = 0;

	if (num_possible_cpus() > 1) {
		core_control_enabled = 1;
		hotplug_enabled = 1;
	}

	key = "qcom,core-limit-temp";
	ret = of_property_read_u32(node, key, &data->core_limit_temp_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,core-temp-hysteresis";
	ret = of_property_read_u32(node, key, &data->core_temp_hysteresis_degC);
	if (ret)
		goto read_node_fail;

	key = "qcom,core-control-mask";
	ret = of_property_read_u32(node, key, &data->core_control_mask);
	if (ret)
		goto read_node_fail;

	key = "qcom,hotplug-temp";
	ret = of_property_read_u32(node, key, &data->hotplug_temp_degC);
	if (ret)
		goto hotplug_node_fail;

	key = "qcom,hotplug-temp-hysteresis";
	ret = of_property_read_u32(node, key,
			&data->hotplug_temp_hysteresis_degC);
	if (ret)
		goto hotplug_node_fail;

	key = "qcom,cpu-sensors";
	cpu_cnt = of_property_count_strings(node, key);
	if (cpu_cnt != num_possible_cpus()) {
		pr_err("%s: Wrong number of cpu\n", KBUILD_MODNAME);
		ret = -EINVAL;
		goto hotplug_node_fail;
	}

	for_each_possible_cpu(cpu) {
		cpus[cpu].cpu = cpu;
		cpus[cpu].offline = 0;
		cpus[cpu].user_offline = 0;
		cpus[cpu].hotplug_thresh_clear = false;
		ret = of_property_read_string_index(node, key, cpu,
				&cpus[cpu].sensor_type);
		if (ret)
			goto hotplug_node_fail;
	}

read_node_fail:
	if (ret) {
		dev_info(&pdev->dev,
			"%s:Failed reading node=%s, key=%s. KTM continues\n",
			KBUILD_MODNAME, node->full_name, key);
		core_control_enabled = 0;
	}

	return ret;

hotplug_node_fail:
	if (ret) {
		dev_info(&pdev->dev,
			"%s:Failed reading node=%s, key=%s. KTM continues\n",
			KBUILD_MODNAME, node->full_name, key);
		hotplug_enabled = 0;
	}

	return ret;
}

static int probe_freq_mitigation(struct device_node *node,
		struct msm_thermal_data *data,
		struct platform_device *pdev)
{
	char *key = NULL;
	int ret = 0;
	uint32_t cpu;

	key = "qcom,freq-mitigation-temp";
	ret = of_property_read_u32(node, key, &data->freq_mitig_temp_degc);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-temp-hysteresis";
	ret = of_property_read_u32(node, key,
		&data->freq_mitig_temp_hysteresis_degc);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-value";
	ret = of_property_read_u32(node, key, &data->freq_limit);
	if (ret)
		goto PROBE_FREQ_EXIT;

	key = "qcom,freq-mitigation-control-mask";
	ret = of_property_read_u32(node, key, &data->freq_mitig_control_mask);
	if (ret)
		goto PROBE_FREQ_EXIT;

	freq_mitigation_enabled = 1;
	for_each_possible_cpu(cpu) {
		cpus[cpu].max_freq = false;
		cpus[cpu].user_max_freq = UINT_MAX;
		cpus[cpu].user_min_freq = 0;
		cpus[cpu].limited_max_freq = UINT_MAX;
		cpus[cpu].limited_min_freq = 0;
		cpus[cpu].freq_thresh_clear = false;
	}

PROBE_FREQ_EXIT:
	if (ret) {
		dev_info(&pdev->dev,
			"%s:Failed reading node=%s, key=%s. KTM continues\n",
			__func__, node->full_name, key);
		freq_mitigation_enabled = 0;
	}
	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	char *key = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;
    
	memset(&data, 0, sizeof(struct msm_thermal_data));
	key = "qcom,sensor-id";
	ret = of_property_read_u32(node, key, &data.sensor_id);
	if (ret)
		goto fail;
	WARN_ON(data.sensor_id >= TSENS_MAX_SENSORS);
    
fail:
	if (ret)
		pr_err("%s: Failed reading node=%s, key=%s\n",
		       __func__, node->full_name, key);
	else
		ret = msm_thermal_init(&data);
    
	return ret;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
	.remove = msm_thermal_dev_exit,
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}
