const $ = (selector) => document.querySelector(selector);
let data = null;
let selectedScenario = "";
let selectedMetric = "mean_tracking_error_m";
let selectedCampaign = "";

const escapeHtml = (value) => String(value ?? "").replace(/[&<>"']/g, (char) => ({"&":"&amp;","<":"&lt;",">":"&gt;","\"":"&quot;","'":"&#39;"}[char]));
const format = (value, digits = 3) => value == null || !Number.isFinite(Number(value)) ? "—" : Number(value).toFixed(digits);
const metricMeta = (key) => data?.metric_labels?.[key] || {label:key, unit:"", direction:"lower"};
const shortScenario = (value) => value.replaceAll("_", " ");

function toast(message) {
  const target = $("#toast");
  target.textContent = message;
  target.classList.add("show");
  window.setTimeout(() => target.classList.remove("show"), 3200);
}

async function fetchOverview() {
  const query = selectedCampaign ? `?campaign=${encodeURIComponent(selectedCampaign)}` : "";
  const response = await fetch(`/api/overview${query}`, {cache:"no-store"});
  if (!response.ok) throw new Error(`API returned ${response.status}`);
  data = await response.json();
  selectedCampaign = data.active_campaign;
  render();
  $("#connection-label").textContent = "本地数据已连接";
  $("#last-updated").textContent = `当前批次 ${data.summary.run_count} 次 · 全库 ${data.summary.overall_run_count} 次`;
}

function renderSummary() {
  const summary = data.summary;
  $("#kpi-runs").textContent = String(summary.run_count).padStart(2, "0");
  $("#kpi-scenes").textContent = String(summary.scenario_count).padStart(2, "0");
  $("#kpi-controllers").textContent = String(summary.controller_count).padStart(2, "0");
  const completion = summary.run_count ? summary.success_count / summary.run_count * 100 : 0;
  $("#kpi-completion").textContent = summary.run_count ? `${completion.toFixed(0)}%` : "—";
  $("#kpi-success-note").textContent = `${summary.success_count} / ${summary.run_count} 次运行成功 · ${summary.telemetry_run_count} 次关联遥测`;
  $("#scene-count").textContent = String(summary.scenario_count).padStart(2, "0");
}

function renderScenarios() {
  const host = $("#scenario-list");
  if (!data.cases.length) {
    host.innerHTML = '<div class="chart-empty">还没有已导入的实验</div>';
    return;
  }
  if (!data.cases.some((item) => item.scenario === selectedScenario)) selectedScenario = data.cases[0].scenario;
  host.innerHTML = data.cases.map((item) => `<button class="scenario-item ${item.scenario === selectedScenario ? "active" : ""}" data-scenario="${escapeHtml(item.scenario)}"><span class="scenario-name">${escapeHtml(shortScenario(item.scenario))}</span><span class="scenario-count">${item.run_count} runs</span></button>`).join("");
  host.querySelectorAll("[data-scenario]").forEach((button) => button.addEventListener("click", () => {
    selectedScenario = button.dataset.scenario;
    renderAnalysis();
    renderScenarios();
  }));
}

function renderCampaigns() {
  const select = $("#campaign-select");
  select.innerHTML = data.campaigns.map((campaign) => `<option value="${escapeHtml(campaign.id)}" ${campaign.id === selectedCampaign ? "selected" : ""}>${escapeHtml(campaign.label)}</option>`).join("");
  select.onchange = async (event) => {
    selectedCampaign = event.target.value;
    selectedScenario = "";
    await fetchOverview();
  };
}

function scenarioCase() {
  return data.cases.find((item) => item.scenario === selectedScenario);
}

function renderMetricOptions(item) {
  const keys = new Set(item?.algorithms.flatMap((algorithm) => Object.keys(algorithm.averages)) || []);
  const preferred = ["mean_tracking_error_m", "average_tracking_error_m", "max_tracking_error_m", "average_speed_mps", "mission_duration_s", ...keys];
  const options = [...new Set(preferred)].filter((key) => keys.has(key));
  if (!options.includes(selectedMetric)) selectedMetric = options[0] || "";
  $("#metric-select").innerHTML = options.map((key) => `<option value="${escapeHtml(key)}" ${key === selectedMetric ? "selected" : ""}>${escapeHtml(metricMeta(key).label)}${metricMeta(key).unit ? ` · ${escapeHtml(metricMeta(key).unit)}` : ""}</option>`).join("");
  $("#metric-select").onchange = (event) => { selectedMetric = event.target.value; renderAnalysis(); };
}

function renderChart(item) {
  const meta = metricMeta(selectedMetric);
  const values = item.algorithms.map((algorithm) => ({algorithm, value:algorithm.averages[selectedMetric]})).filter((entry) => entry.value != null && Number.isFinite(Number(entry.value)));
  $("#metric-description").textContent = `按 ${meta.direction === "higher" ? "越高越优" : "越低越优"} 解读；仅展示成功运行的均值。`;
  if (!values.length) {
    $("#chart").innerHTML = '<div class="chart-empty">该场景暂无此指标的有效数据</div>';
    $("#scale-low").textContent = "无数据";
    $("#scale-high").textContent = "";
    return;
  }
  const low = Math.min(...values.map((entry) => Number(entry.value)));
  const high = Math.max(...values.map((entry) => Number(entry.value)));
  const max = high > 0 ? high : 1;
  $("#scale-low").textContent = meta.direction === "higher" ? format(low) : format(high);
  $("#scale-high").textContent = meta.direction === "higher" ? format(high) : format(low);
  $("#chart").innerHTML = values.map(({algorithm, value}) => {
    const ratio = high === low ? 74 : Math.max(6, Number(value) / max * 100);
    const favorable = meta.direction === "higher" ? Number(value) === high : Number(value) === low;
    const shown = meta.unit === "%" ? `${(Number(value) * 100).toFixed(1)}%` : `${format(value, 4)} ${meta.unit}`;
    return `<div class="bar-row"><span class="bar-label">${escapeHtml(algorithm.name)}</span><div class="bar-track"><div class="bar-fill ${favorable ? "" : "bad"}" style="width:${Math.min(100, ratio)}%"></div></div><span class="bar-value">${escapeHtml(shown)}</span></div>`;
  }).join("");
}

function renderCards(item) {
  const meta = metricMeta(selectedMetric);
  $("#controller-grid").innerHTML = item.algorithms.map((algorithm) => {
    const value = algorithm.averages[selectedMetric];
    const spread = algorithm.stddev[selectedMetric];
    const displayValue = meta.unit === "%" && value != null ? `${(value * 100).toFixed(1)}%` : format(value, 4);
    const displayStd = spread == null ? "" : ` ± ${format(spread, 4)}`;
    const readiness = algorithm.repeat_ready ? "重复数 ≥ 3" : `成功 ${algorithm.success_count} 次`;
    return `<article class="controller-card"><div class="controller-top"><span class="controller-name">${escapeHtml(algorithm.name)}</span><span class="controller-runs">n=${algorithm.run_count}</span></div><div class="controller-value">${escapeHtml(displayValue)}<span class="controller-unit">${meta.unit === "%" ? "" : escapeHtml(meta.unit)}</span></div><div class="controller-caption">${escapeHtml(meta.label)}${escapeHtml(displayStd)}</div><div class="completion-line"><span style="width:${Math.round(algorithm.completion_rate * 100)}%"></span></div><div class="controller-caption">完成率 ${Math.round(algorithm.completion_rate * 100)}% · ${escapeHtml(readiness)}</div><p class="controller-description">${escapeHtml(algorithm.description)}</p></article>`;
  }).join("");
}

function renderInsights(item) {
  $("#insight-list").innerHTML = item.explanations.map((line) => `<li>${escapeHtml(line)}</li>`).join("");
}

function renderAnalysis() {
  const item = scenarioCase();
  if (!item) return;
  $("#comparison-title").textContent = `${shortScenario(item.scenario)} · 算法表现`;
  renderMetricOptions(item);
  renderChart(item);
  renderCards(item);
  renderInsights(item);
}

function reportMetric(run) {
  const metrics = run.metrics || {};
  for (const key of ["mean_tracking_error_m", "average_tracking_error_m", "max_tracking_error_m", "average_speed_mps"]) {
    if (metrics[key] != null) {
      const meta = metricMeta(key);
      return `${meta.label} ${format(metrics[key], 4)} ${meta.unit}`;
    }
  }
  return run.mission_duration_s == null ? "暂无指标" : `耗时 ${format(run.mission_duration_s, 2)} s`;
}

function renderRuns() {
  const runs = data.runs.slice(0, 30);
  $("#runs-body").innerHTML = runs.length ? runs.map((run) => {
    const telemetryCount = run.metrics.telemetry_sample_count || 0;
    const state = run.status === "success" ? "成功" : run.status === "failed" ? "失败" : "未知";
    return `<tr><td class="run-id" title="${escapeHtml(run.run_id)}">${escapeHtml(run.run_id)}</td><td>${escapeHtml(shortScenario(run.scenario))}</td><td><span class="controller-tag">${escapeHtml((data.controllers[run.controller] || data.controllers.unknown).name)}</span></td><td><span class="status ${escapeHtml(run.status)}">${state}</span></td><td>${escapeHtml(reportMetric(run))}</td><td>${telemetryCount ? `<span class="telemetry-tag">${telemetryCount} samples</span>` : '<span class="telemetry-missing">无关联遥测</span>'}</td></tr>`;
  }).join("") : '<tr><td colspan="6" class="chart-empty">暂无运行报告</td></tr>';
}

function render() {
  renderCampaigns();
  renderSummary();
  renderScenarios();
  renderAnalysis();
  renderRuns();
}

$("#import-button").addEventListener("click", () => $("#file-input").click());
$("#file-input").addEventListener("change", async (event) => {
  const files = [...event.target.files];
  for (const file of files) {
    try {
      const response = await fetch("/api/import", {method:"POST", headers:{"Content-Type":"application/json", "X-Filename":encodeURIComponent(file.name)}, body:await file.text()});
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || "导入失败");
      toast(`已导入 ${result.run_id} · 遥测 ${result.telemetry_samples} 条`);
    } catch (error) {
      toast(`${file.name}：${error.message}`);
    }
  }
  event.target.value = "";
  await fetchOverview();
});

$("#sync-button").addEventListener("click", async () => {
  try {
    const response = await fetch("/api/sync-telemetry", {method:"POST"});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "同步失败");
    await fetchOverview();
    toast(`遥测已同步，共导入 ${result.telemetry_samples} 个样本`);
  } catch (error) { toast(error.message); }
});

fetchOverview().catch((error) => {
  $("#connection-label").textContent = "服务连接失败";
  $("#last-updated").textContent = error.message;
  toast("无法读取分析数据，请确认后端服务已启动");
});
