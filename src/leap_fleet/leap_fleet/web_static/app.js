"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const BUSY = new Set(["starting", "running", "stopping"]);
  const STATUS = {
    connected: ["连接正常", "connected"],
    discovered: ["接口待确认", "discovered"],
    stale: ["数据过期", "stale"],
    offline: ["未连接", "offline"],
    error: ["连接异常", "error"],
  };
  const JOB_STATUS = {
    starting: "任务启动中", running: "任务执行中", stopping: "正在停车 · 等待反馈",
    completed: "里程计停车反馈已确认 · 任务完成", stopped: "里程计停车反馈已确认 · 任务已停止",
    unconfirmed: "里程计停车反馈未确认", failed: "任务执行失败",
  };
  const MOCAP_STATUS = {
    fresh: "新鲜数据", stale: "数据过期", missing: "未收到", invalid: "数据无效", error: "接入异常",
  };
  const CONNECTION_PHASES = {
    idle: ["未检测", "idle"], checking: ["检测中", "working"], needs_config: ["需应用配置", "warning"],
    starting: ["启动代理", "working"], configuring: ["配置并重启中", "working"],
    waiting_raw: ["等待底盘数据", "working"], starting_base: ["启动融合节点", "working"],
    waiting_filtered: ["等待融合数据", "working"], connected: ["已连接", "ready"],
    external: ["外部已连接", "external"], blocked: ["操作受阻", "warning"],
    waiting_data: ["等待数据", "warning"], stopping: ["关闭连接中", "working"],
    stopped: ["连接已关闭", "idle"], failed: ["连接失败", "failed"],
  };
  const state = {
    snapshot: null, online: false, receivedAt: 0, selected: new Set(), mode: "uniform",
    individual: new Map(), cards: new Map(), runPending: false, stopPending: false,
    polling: false, timer: null, localEvents: [], rowsKey: "", eventKey: "", csrf: "",
    lastJobKey: "", unknownOutcome: false, unknownSince: 0, connectionError: "",
    mocapFrame: null, mocapFramesKey: "", mocapBounds: new Map(),
    connectionSelected: new Set(), connectionSelectionInitialized: false, connectionCards: new Map(),
    connectionPending: false, connectionMutationTargets: new Set(), connectionUnknown: false, connectionUnknownSince: 0,
    stopResultsKey: "",
    controlMode: "velocity", trajectory: null, trajectorySelected: new Set(),
    trajectoryCheck: null, trajectoryCheckKey: "", trajectoryCheckedAt: 0,
    trajectoryBusy: "", trajectoryRevision: 0, trajectoryIgnoredIds: new Set(),
    trajectoryFetchId: "", trajectoryFetching: false, trajectoryRowsKey: "",
    trajectoryPreviewFrameKey: null,
    trajectoryImportsInFlight: new Set(),
  };

  const cars = () => state.snapshot?.cars || [];
  const job = () => state.snapshot?.job || null;
  const busy = () => state.runPending || (job() && BUSY.has(job().status));
  const activeJob = () => job() && BUSY.has(job().status) ? job() : null;
  const displayedIds = () => activeJob() ? new Set(activeJob().cars || [])
    : state.controlMode === "trajectory" ? state.trajectorySelected : state.selected;
  const isConnected = (car) => car.status === "connected";
  const finite = (value) => value !== "" && Number.isFinite(Number(value));
  const number = (value, digits = 3) => typeof value === "number" && Number.isFinite(value)
    ? value.toFixed(digits) : "—";
  const signed = (value) => `${value > 0 ? "+" : ""}${number(value)}`;
  const numeric = (value) => typeof value === "number" && Number.isFinite(value);
  const connections = () => state.snapshot?.connections || null;
  const connectionRows = () => Array.isArray(connections()?.cars) ? connections().cars : [];
  const connectionAvailable = () => connections()?.available === true;
  const trajectoryCars = () => Array.isArray(state.trajectory?.cars) ? state.trajectory.cars : [];
  const trajectoryKey = () => JSON.stringify([state.trajectory?.id, [...state.trajectorySelected].sort(), state.trajectoryRevision]);
  const trajectoryColor = (id) => ["#397aa7", "#b07d3c", "#8b65a5", "#159584", "#c36372", "#6c8b47", "#5477b0", "#b76f43"][Math.abs([...String(id)].reduce((sum, char) => sum + char.charCodeAt(0), 0)) % 8];

  function trajectoryFrameInfo() {
    const checkedFrame = state.trajectoryCheckKey === trajectoryKey() ? state.trajectoryCheck?.frame_id : null;
    if (typeof checkedFrame === "string" && checkedFrame.trim()) return { key: checkedFrame, label: checkedFrame, known: true };
    const ids = [...state.trajectorySelected];
    const selected = cars().filter((car) => state.trajectorySelected.has(car.id)).map(mocapView);
    const frames = new Set(selected.map((view) => view.frame));
    if (ids.length && selected.length === ids.length && selected.every((view) => view.fresh && view.frame) && frames.size === 1) {
      const frame = [...frames][0]; return { key: frame, label: frame, known: true };
    }
    return { key: "@csv-world-preview-unverified", label: "CSV world 独立预览（坐标系待核对）", known: false };
  }

  function trajectoryMessage(text, error = false, warning = false) {
    $("trajectory-message").textContent = text;
    $("trajectory-message").classList.toggle("error", error);
    $("trajectory-message").classList.toggle("warning", warning && !error);
    $("trajectory-message").hidden = !text;
  }

  function trajectoryCheckInfo(record) {
    const messages = (value) => Array.isArray(value) ? value.filter((item) => typeof item === "string" && item) : [];
    const reasons = messages(record?.reasons);
    const modern = Array.isArray(record?.warnings) || Array.isArray(record?.errors);
    const warnings = [...new Set(modern ? messages(record?.warnings) : record?.ready === false ? reasons : [])];
    const errors = [...new Set(messages(record?.errors))];
    return { warnings, errors, reasons: reasons.filter((text) => !warnings.includes(text) && !errors.includes(text)) };
  }

  function clearTrajectoryCheck() {
    state.trajectoryCheck = null; state.trajectoryCheckKey = ""; state.trajectoryCheckedAt = 0;
  }

  function clearTrajectory() {
    if (state.trajectory?.id) state.trajectoryIgnoredIds.add(state.trajectory.id);
    if (state.snapshot?.trajectory?.id) state.trajectoryIgnoredIds.add(state.snapshot.trajectory.id);
    state.trajectoryRevision++;
    state.trajectory = null; state.trajectorySelected.clear(); state.trajectoryRowsKey = "";
    if (state.mocapFrame === "@csv-world-preview-unverified") state.mocapFrame = null;
    state.trajectoryPreviewFrameKey = null;
    state.trajectoryFetchId = "";
    clearTrajectoryCheck();
    state.mocapBounds.delete("world");
  }

  function acceptTrajectory(trajectory) {
    if (!trajectory || typeof trajectory.id !== "string" || !Array.isArray(trajectory.cars) || !trajectory.cars.length
        || trajectory.cars.some((car) => typeof car.car_id !== "string" || !Array.isArray(car.points) || car.points.length < 2
          || car.points.some((point) => !numeric(point.x) || !numeric(point.y) || !numeric(point.t)))) throw new Error("服务返回的轨迹预览格式无效或缺少绘图点");
    state.trajectory = trajectory;
    state.trajectorySelected = new Set(trajectory.cars.map((car) => car.car_id));
    state.trajectoryIgnoredIds.delete(trajectory.id);
    state.trajectoryFetchId = trajectory.id;
    state.trajectoryRowsKey = "";
    clearTrajectoryCheck();
    const frame = trajectoryFrameInfo();
    if (state.controlMode === "trajectory") state.mocapFrame = frame.key;
    state.mocapBounds.delete(frame.key);
  }

  function downloadText(text, filename, type = "text/csv;charset=utf-8") {
    const url = URL.createObjectURL(new Blob([text], { type }));
    const anchor = document.createElement("a"); anchor.href = url; anchor.download = filename; anchor.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  async function importTrajectory(csv, filename, revision) {
    if (typeof csv !== "string" || !csv.trim()) throw new Error("CSV 文件为空");
    if (new Blob([csv]).size > 8 * 1024 * 1024) throw new Error("CSV 超过 8 MiB 上限");
    state.trajectoryImportsInFlight.add(revision);
    try {
      const response = await request("/api/trajectory/import", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: JSON.stringify({ csv, filename }) }, 15000);
      if (revision !== state.trajectoryRevision) {
        if (response.trajectory?.id) state.trajectoryIgnoredIds.add(response.trajectory.id);
        return;
      }
      acceptTrajectory(response.trajectory);
      trajectoryMessage(`已导入 ${response.trajectory.filename}，默认选择文件内 ${trajectoryCars().length} 辆车。当前仅预览，尚未运动。`);
      log(`CSV仅预览导入：${response.trajectory.filename}，${trajectoryCars().length} 辆车。`);
    } finally { state.trajectoryImportsInFlight.delete(revision); }
  }

  async function importTrajectoryFile() {
    if (busy()) return;
    const file = $("trajectory-file").files?.[0];
    clearTrajectory();
    state.controlMode = "trajectory";
    const revision = state.trajectoryRevision;
    if (!file) { state.trajectoryBusy = ""; trajectoryMessage(""); render(); return; }
    state.trajectoryBusy = "import";
    trajectoryMessage(`正在读取 ${file.name}；旧预览与旧准备结果已清除。`);
    render();
    try {
      if (!state.online || !state.csrf) throw new Error("控制服务未连接，不能导入");
      if (file.size > 8 * 1024 * 1024) throw new Error("CSV 超过 8 MiB 上限");
      const csv = await file.text();
      if (revision !== state.trajectoryRevision) return;
      await importTrajectory(csv, file.name, revision);
    } catch (error) {
      if (revision === state.trajectoryRevision) trajectoryMessage(`导入失败：${error.message}。旧文件已清除，不会执行旧轨迹。`, true);
    } finally {
      if (revision === state.trajectoryRevision) state.trajectoryBusy = "";
      render();
    }
  }

  async function hydrateTrajectory(id = null, force = false) {
    if (state.trajectoryFetching || state.trajectoryBusy || state.trajectoryImportsInFlight.size || !state.online || (busy() && force)) return;
    if (!force && (!id || state.trajectoryIgnoredIds.has(id) || state.trajectory?.id === id || state.trajectoryFetchId === id)) return;
    if (force) clearTrajectory();
    const revision = state.trajectoryRevision;
    state.trajectoryFetching = true;
    state.trajectoryFetchId = id || "manual";
    renderTrajectoryControls();
    try {
      const response = await request("/api/trajectory", {}, 10000);
      if (revision !== state.trajectoryRevision) return;
      if (response.trajectory) {
        if (id && response.trajectory.id !== id) return;
        if (!force && state.trajectoryIgnoredIds.has(response.trajectory.id)) return;
        acceptTrajectory(response.trajectory);
        trajectoryMessage(`已恢复 ${response.trajectory.filename} 的只读预览；状态检查可选，不是开始条件。`);
      } else trajectoryMessage("服务端暂无 CSV 轨迹，请选择文件导入。");
    } catch (error) {
      if (revision === state.trajectoryRevision) trajectoryMessage(`轨迹预览未加载：${error.message}。可点击“重载服务端预览”重试。`, true);
    } finally { state.trajectoryFetching = false; render(); }
  }

  async function generateTrajectory() {
    if (busy() || state.trajectoryBusy || !state.online || !state.csrf) return;
    const ids = state.trajectorySelected.size ? [...state.trajectorySelected] : [...state.selected];
    const distance = Number($("trajectory-generate-distance").value), duration = Number($("trajectory-generate-duration").value);
    if (!ids.length) { trajectoryMessage("生成前请勾选 CSV 参与车，或先在左侧选择速度车辆。", true); return; }
    if (!finite($("trajectory-generate-distance").value) || !finite($("trajectory-generate-duration").value) || distance < 0.01 || distance > 5 || duration < 1 || duration > 3600) { trajectoryMessage("生成距离须为 0.01–5 m，时长须为 1–3600 s。", true); return; }
    clearTrajectory();
    const revision = state.trajectoryRevision;
    state.trajectoryBusy = "generate";
    trajectoryMessage("正在从当前动捕位姿生成直线 CSV；只生成文件，不运动。");
    render();
    try {
      const response = await request("/api/trajectory/generate", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: JSON.stringify({ cars: ids, distance, duration }) }, 10000);
      if (revision !== state.trajectoryRevision) return;
      if (typeof response.csv !== "string" || typeof response.filename !== "string") throw new Error("生成结果缺少 CSV 文件");
      downloadText(response.csv, response.filename);
      await importTrajectory(response.csv, response.filename, revision);
    } catch (error) {
      if (revision === state.trajectoryRevision) trajectoryMessage(`生成或导入失败：${error.message}。没有发送运动命令。`, true);
    } finally { if (revision === state.trajectoryRevision) state.trajectoryBusy = ""; render(); }
  }

  async function checkTrajectory() {
    if (busy() || state.trajectoryBusy || !state.online || !state.csrf || !state.trajectory || !state.trajectorySelected.size) return;
    const key = trajectoryKey();
    const payload = { trajectory_id: state.trajectory.id, cars: [...state.trajectorySelected], operator_confirmed: false };
    clearTrajectoryCheck(); state.trajectoryBusy = "check"; render();
    try {
      const response = await request("/api/trajectory/check", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: JSON.stringify(payload) }, 10000);
      if (trajectoryKey() !== key) return;
      state.trajectoryCheck = response; state.trajectoryCheckKey = key; state.trajectoryCheckedAt = Date.now();
      const group = trajectoryCheckInfo(response);
      const rows = (Array.isArray(response.cars) ? response.cars : []).map(trajectoryCheckInfo);
      const errors = [...new Set([...group.errors, ...rows.flatMap((row) => row.errors)])];
      const warnings = [...new Set([...group.warnings, ...rows.flatMap((row) => row.warnings)])];
      if (errors.length) trajectoryMessage(`发现输入或资源问题：${errors.join("；")}。检查不锁定按钮，开始时服务端仍会复核。`, true);
      else if (warnings.length) trajectoryMessage(`风险提示 · 可启动：${warnings.join("；")}。是否具备可用输入仍由开始请求判断。`, false, true);
      else trajectoryMessage("状态检查已完成；尚未运动。检查为可选提示，点击开始才会提交任务。");
    } catch (error) { if (trajectoryKey() === key) trajectoryMessage(`状态检查未完成：${error.message}。此检查不锁定开始按钮。`, true); }
    finally { if (state.trajectoryBusy === "check") state.trajectoryBusy = ""; render(); }
  }

  function trajectoryDraft() {
    if (!state.online || Date.now() - state.receivedAt > 2000) return { error: "控制服务未连接或状态已过期，轨迹开始已禁用。" };
    if (busy()) return { error: "任务执行期间配置只读；停止按钮始终可用。" };
    if (state.unknownOutcome || state.connectionUnknown) return { error: "上次操作结果待确认，请等待状态恢复后再开始。" };
    if ((state.trajectoryBusy && state.trajectoryBusy !== "check") || state.trajectoryFetching || state.trajectoryImportsInFlight.size) return { error: "正在处理 CSV 文件，请等待解析完成。" };
    if (!state.trajectory) return { error: "请先导入一个 CSV；导入不会自动运动。" };
    const ids = [...state.trajectorySelected];
    if (!ids.length) return { error: "请勾选至少一辆 CSV 参与车辆。" };
    const manifest = new Set(trajectoryCars().map((car) => car.car_id));
    if (ids.some((id) => !manifest.has(id))) return { error: "选择中存在没有本文件轨迹的车辆，请重新选择。" };
    return { trajectory_id: state.trajectory.id, cars: ids, operator_confirmed: false };
  }

  function renderTrajectoryControls() {
    const locked = Boolean(busy());
    const processing = Boolean(state.trajectoryBusy) || state.trajectoryFetching;
    $("trajectory-file").disabled = locked;
    $("trajectory-check").disabled = locked || processing || !state.online || !state.csrf || !state.trajectory || !state.trajectorySelected.size;
    $("trajectory-check").textContent = state.trajectoryBusy === "check" ? "正在检查状态…" : "检查轨迹状态（仅提示）";
    $("trajectory-select-all").disabled = locked || processing || !trajectoryCars().length;
    $("trajectory-clear").disabled = locked || processing || !state.trajectorySelected.size;
    $("trajectory-reload").disabled = locked || processing || !state.online;
    $("trajectory-generate").disabled = locked || processing || !state.online || !state.csrf;
    $("trajectory-generate-distance").disabled = locked || processing;
    $("trajectory-generate-duration").disabled = locked || processing;
    $("trajectory-file-summary").textContent = state.trajectory
      ? `${state.trajectory.filename} · ${state.trajectory.sample_count ?? "—"} 点 · ${trajectoryCars().length} 辆 · 最长 ${number(state.trajectory.duration, 2)} s`
      : state.trajectoryBusy ? "正在处理文件；旧预览已清除" : state.trajectoryFetching ? "正在恢复服务端预览" : "尚未导入 CSV";
    const key = state.trajectory?.id || "empty";
    if (state.trajectoryRowsKey !== key) {
      state.trajectoryRowsKey = key;
      $("trajectory-car-list").replaceChildren();
      if (!trajectoryCars().length) {
        const empty = document.createElement("p"); empty.className = "small-empty"; empty.textContent = "导入后显示匹配车辆、点数与各车时长。"; $("trajectory-car-list").append(empty);
      }
      trajectoryCars().forEach((car) => {
        const row = document.createElement("article"); row.className = "trajectory-car"; row.dataset.carId = car.car_id;
        row.innerHTML = '<div class="trajectory-car-top"><label><input type="checkbox"><i></i><strong></strong></label><span class="trajectory-ready-state"></span></div><p class="trajectory-car-metadata"></p><p class="trajectory-car-reasons"></p><p class="trajectory-car-warnings"></p><p class="trajectory-car-errors"></p>';
        row.querySelector("strong").textContent = car.car_id;
        row.querySelector("i").style.backgroundColor = trajectoryColor(car.car_id);
        const checkbox = row.querySelector("input"); checkbox.setAttribute("aria-label", `CSV参与车辆 ${car.car_id}`);
        checkbox.addEventListener("change", () => {
          if (busy() || state.trajectoryBusy) return;
          if (checkbox.checked) state.trajectorySelected.add(car.car_id); else state.trajectorySelected.delete(car.car_id);
          clearTrajectoryCheck(); render();
        });
        row.querySelector(".trajectory-car-metadata").textContent = `${car.sample_count ?? car.points?.length ?? "—"} 点 · ${number(car.duration, 2)} s · max ${number(car.max_speed)} m/s`;
        $("trajectory-car-list").append(row);
      });
    }
    const checks = new Map((Array.isArray(state.trajectoryCheck?.cars) ? state.trajectoryCheck.cars : []).map((car) => [car.car_id, car]));
    $("trajectory-car-list").querySelectorAll(".trajectory-car").forEach((row) => {
      const checkbox = row.querySelector("input"); checkbox.checked = state.trajectorySelected.has(row.dataset.carId); checkbox.disabled = locked || processing;
      const check = checks.get(row.dataset.carId);
      const info = trajectoryCheckInfo(check);
      const badge = row.querySelector(".trajectory-ready-state");
      badge.textContent = !check ? "检查可选" : info.errors.length ? "输入 / 资源问题" : info.warnings.length ? "风险提示 · 可启动" : check.ready ? "输入可用" : "状态待复核";
      badge.classList.toggle("ready", check?.ready === true && !info.errors.length && !info.warnings.length);
      badge.classList.toggle("warning", info.warnings.length > 0 && !info.errors.length);
      badge.classList.toggle("error", info.errors.length > 0);
      row.querySelector(".trajectory-car-reasons").textContent = info.reasons.join("；");
      row.querySelector(".trajectory-car-warnings").textContent = info.warnings.length ? `风险：${info.warnings.join("；")}` : "";
      row.querySelector(".trajectory-car-errors").textContent = info.errors.length ? `输入 / 资源：${info.errors.join("；")}` : "";
      row.classList.toggle("selected", checkbox.checked);
    });
    const group = trajectoryCheckInfo(state.trajectoryCheck);
    $("trajectory-check-status").textContent = state.trajectoryCheck
      ? `状态快照 · ${clock(state.trajectoryCheckedAt)}。${[...group.reasons, ...group.warnings, ...group.errors].join("；") || "逐车信息见上方。"} 检查仅提示，不锁定开始；实际输入可用性由服务端执行请求判断。`
      : "检查可选，只提示风险，不锁定开始按钮；开始时服务端校验可用输入与任务占用。";
  }

  async function runTrajectory() {
    const draft = trajectoryDraft();
    if (draft.error || !state.csrf) return;
    const payload = JSON.stringify(draft);
    state.runPending = true; state.unknownOutcome = false; message(""); render();
    log(`提交CSV任务：${state.trajectory.filename} · ${draft.cars.join("、")}。`);
    try {
      const response = await request("/api/trajectory/run", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: payload });
      if (response.job) state.snapshot.job = response.job;
      message("CSV任务已提交；共同计时与逐车结束状态将在任务反馈中更新。");
    } catch (error) {
      clearTrajectoryCheck();
      state.unknownOutcome = !error.httpStatus || error.httpStatus >= 500; state.unknownSince = Date.now();
      if (!error.httpStatus) { state.online = false; state.connectionError = error.message; }
      message(state.unknownOutcome ? `CSV开始请求结果未确认：${error.message}。不会自动重试，请查看任务状态或停止。` : `CSV任务未启动：${error.message}。可用输入或任务资源不满足服务端执行条件。`, true);
    } finally { state.runPending = false; render(); poll(); }
  }

  function stopResults(current) {
    const results = current?.car_results && typeof current.car_results === "object" ? current.car_results : {};
    const ids = [...new Set([...(Array.isArray(current?.cars) ? current.cars : []), ...Object.keys(results)])];
    return ids.map((id) => {
      const result = results[id] || {};
      const status = ["pending", "confirmed", "unconfirmed"].includes(result.stop_status)
        ? result.stop_status : BUSY.has(current?.status) ? "pending" : "unconfirmed";
      const fallback = status === "confirmed" ? "停止后连续新鲜里程计反馈近零"
        : status === "pending" ? "等待停止后的连续新鲜里程计反馈" : "尚未提供可确认停车的逐车里程计反馈";
      return { id, status, reason: result.reason || fallback, error: result.error || "", commandCount: result.command_count, zeroCount: result.zero_count };
    });
  }

  function jobLabel(current) {
    if (!current) return "暂无任务";
    if (["completed", "stopped"].includes(current.status)) {
      const results = stopResults(current);
      if (!results.length || results.some((result) => result.status !== "confirmed")) return "里程计停车反馈未确认 · 反馈不完整";
    }
    return JOB_STATUS[current.status] || current.status;
  }

  function renderStopResults(current) {
    $("job-stop-results").hidden = !current;
    if (!current) return;
    const results = stopResults(current);
    const confirmed = results.filter((result) => result.status === "confirmed").length;
    const pending = results.filter((result) => result.status === "pending").length;
    const unconfirmed = results.filter((result) => result.status === "unconfirmed").length;
    const warning = current.status === "unconfirmed" || unconfirmed > 0 || (!BUSY.has(current.status) && (!results.length || confirmed !== results.length));
    $("job-stop-results").classList.toggle("warning", warning);
    $("job-stop-results").classList.toggle("cached", !state.online);
    $("job-stop-title").textContent = current.kind === "stop_all" ? "全车里程计停车反馈" : "逐车里程计停车反馈";
    $("job-stop-summary").textContent = results.length ? `已确认 ${confirmed} · 未确认 ${unconfirmed} · 待确认 ${pending}` : "尚无逐车反馈";
    $("job-stop-note").textContent = state.online
      ? "确认依据：停止后的连续新鲜里程计近零反馈；并非固件 ACK。"
      : "服务失联，以下为上次反馈结果，不能确认当前状态；停车确认依据里程计反馈，并非固件 ACK。";
    const key = JSON.stringify([current.id, current.status, results, state.online]);
    if (state.stopResultsKey === key) return;
    state.stopResultsKey = key;
    $("job-stop-rows").replaceChildren();
    if (!results.length) {
      const empty = document.createElement("p"); empty.className = "stop-results-empty";
      empty.textContent = "服务未提供参与车辆或停车反馈，不能据任务结束判断已停车。";
      $("job-stop-rows").append(empty);
    }
    results.forEach((result) => {
      const row = document.createElement("article"); row.className = `stop-result ${result.status}`;
      row.dataset.carId = result.id; row.dataset.stopStatus = result.status;
      const heading = document.createElement("div"); heading.className = "stop-result-heading";
      const id = document.createElement("strong"); id.textContent = result.id;
      const badge = document.createElement("span"); badge.className = "stop-result-badge";
      badge.textContent = `${state.online ? "" : "上次："}${{ confirmed: "里程计近零", unconfirmed: "里程计反馈未确认", pending: "等待里程计反馈" }[result.status]}`;
      heading.append(id, badge);
      const reason = document.createElement("p"); reason.className = "stop-result-reason"; reason.textContent = result.reason;
      const counts = document.createElement("p"); counts.className = "stop-result-counts";
      counts.textContent = `发布计数：命令 ${numeric(result.commandCount) ? result.commandCount : "—"} · 零速度 ${numeric(result.zeroCount) ? result.zeroCount : "—"}`;
      row.append(heading, reason, counts);
      if (result.error && result.error !== result.reason) {
        const error = document.createElement("p"); error.className = "stop-result-error"; error.textContent = result.error; row.append(error);
      }
      $("job-stop-rows").append(row);
    });
  }

  function connectionTargetsBusy(ids) {
    return connectionRows().filter((row) => ids.has(row.id) && row.busy).map((row) => row.id)
      .concat([...state.connectionMutationTargets].filter((id) => ids.has(id)));
  }

  function connectionFeedback(text, error = false) {
    $("connections-feedback").textContent = text;
    $("connections-feedback").classList.toggle("error", error);
    $("connections-feedback").hidden = !text;
  }

  function connectionRuntime(value) {
    if (!value || typeof value !== "object") return "尚未读取";
    return `${value.comm_mode ?? "—"} · ${value.microros_agent_ip ?? "—"}:${value.microros_agent_port ?? "—"}`;
  }

  function createConnectionCard(car) {
    const card = document.createElement("article");
    card.className = "connection-card";
    card.dataset.carId = car.id;
    card.setAttribute("role", "listitem");
    card.innerHTML = '<div class="connection-card-top"><label class="connection-choice"><input type="checkbox"><strong></strong></label><span class="connection-phase"></span></div><div class="connection-card-ip"></div><p class="connection-message"></p><p class="connection-error" hidden></p><div class="connection-config" hidden><div><span>当前</span><code class="connection-current"></code></div><div><span>目标</span><code class="connection-target"></code></div><button class="button connection-configure" type="button">应用配置并连接（会重启）</button></div><div class="connection-card-actions"><span class="connection-owner"></span><button class="text-button connection-cancel" type="button" hidden>关闭本页连接</button></div><details class="connection-log"><summary>配置与日志详情</summary><div class="connection-log-runtime"><span>当前配置</span><code class="connection-current-detail"></code><span>目标配置</span><code class="connection-target-detail"></code></div><p class="connection-log-file"></p><pre class="connection-log-tail"></pre></details>';
    const watchdog = document.createElement("section");
    watchdog.className = "connection-watchdog";
    watchdog.setAttribute("aria-label", "车端超时停车检测快照");
    watchdog.innerHTML = '<strong class="watchdog-summary">车端超时停车：未报告</strong><span class="watchdog-snapshot">尚无检测快照 · 非实时</span><p class="watchdog-timed-out" hidden></p><p class="watchdog-check-error" hidden></p>';
    card.querySelector(".connection-error").before(watchdog);
    const details = document.createElement("div");
    details.className = "watchdog-details";
    details.innerHTML = '<p class="watchdog-firmware"></p><p class="watchdog-command-age"></p><p class="watchdog-sequences"></p><p class="watchdog-last-check"></p>';
    card.querySelector(".connection-log-runtime").before(details);
    card.querySelector(".connection-log summary").textContent = "车端状态、配置与日志";
    const checkbox = card.querySelector("input");
    checkbox.setAttribute("aria-label", `连接目标 ${car.id}，与运动选择独立`);
    checkbox.addEventListener("change", () => {
      if (checkbox.checked) state.connectionSelected.add(car.id);
      else state.connectionSelected.delete(car.id);
      renderConnections();
    });
    card.querySelector("strong").textContent = car.id;
    card.querySelector(".connection-configure").addEventListener("click", () => connectionAction("configure", [car.id]));
    card.querySelector(".connection-cancel").addEventListener("click", () => connectionAction("cancel", [car.id]));
    $("connection-grid").append(card);
    state.connectionCards.set(car.id, card);
    return card;
  }

  function renderConnectionWatchdog(card, row) {
    const metadata = row.vehicle_status && typeof row.vehicle_status === "object" ? row.vehicle_status : {};
    const watchdog = metadata.cmd_vel_watchdog && typeof metadata.cmd_vel_watchdog === "object" ? metadata.cmd_vel_watchdog : {};
    const firmware = metadata.firmware && typeof metadata.firmware === "object" ? metadata.firmware : {};
    const checked = numeric(metadata.checked_at_unix) && metadata.checked_at_unix > 0;
    const historical = !state.online || ["checking", "failed", "cancelled"].includes(metadata.check_state);
    const validInteger = (value) => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
    let label = "未报告";
    if (watchdog.enabled === true) label = `已启用 · ${validInteger(watchdog.timeout_ms) ? `${watchdog.timeout_ms} ms` : "阈值未报告"}`;
    else if (watchdog.enabled === false) label = "未启用";
    const section = card.querySelector(".connection-watchdog");
    section.classList.toggle("historical", historical);
    section.classList.toggle("enabled", watchdog.enabled === true && !historical);
    section.classList.toggle("not-enabled", watchdog.enabled === false && !historical);
    section.dataset.checkState = metadata.check_state || "never";
    card.querySelector(".watchdog-summary").textContent = `${historical && checked ? "上次：" : ""}车端超时停车：${label}`;
    let caption = checked ? `检测快照 ${clock(metadata.checked_at_unix)} · 非实时` : "尚无成功检测快照 · 非实时";
    if (metadata.check_state === "checking") caption = checked ? "正在重新检测 · 下方为上次快照" : "正在检测 · 尚无成功快照";
    else if (metadata.check_state === "failed") caption = checked ? "最近检测失败 · 保留上次成功快照" : "检测失败 · 尚无成功快照";
    else if (metadata.check_state === "cancelled") caption = checked ? "本次检测已取消 · 保留上次快照" : "检测已取消 · 尚无成功快照";
    if (!state.online && checked) caption = "服务失联 · 保留上次检测快照（非实时）";
    card.querySelector(".watchdog-snapshot").textContent = caption;
    card.querySelector(".watchdog-timed-out").hidden = watchdog.timed_out !== true;
    card.querySelector(".watchdog-timed-out").textContent = "最后成功检测时：速度指令已到期。任务结束后不再发送速度也会出现；这是检测快照，不代表通信故障。";
    card.querySelector(".watchdog-check-error").hidden = !metadata.error;
    card.querySelector(".watchdog-check-error").textContent = metadata.error || "";
    card.querySelector(".watchdog-firmware").textContent = `固件 ${typeof firmware.version === "string" && firmware.version ? firmware.version : "未报告"} · 运动安全修订 ${validInteger(firmware.motion_safety_revision) ? firmware.motion_safety_revision : "—"}`;
    const age = validInteger(watchdog.last_cmd_age_ms) ? `${watchdog.last_cmd_age_ms} ms` : watchdog.has_command === false ? "尚无命令" : "未报告";
    const hasCommand = watchdog.has_command === true ? "有" : watchdog.has_command === false ? "无" : "未报告";
    card.querySelector(".watchdog-command-age").textContent = `检测时命令记录：${hasCommand} · 年龄：${age} · 超时状态：${watchdog.timed_out === true ? "已超时" : watchdog.timed_out === false ? "未超时" : "未报告"}`;
    card.querySelector(".watchdog-sequences").textContent = `车端接收序号 ${validInteger(watchdog.received_sequence) ? watchdog.received_sequence : "—"} · 应用序号 ${validInteger(watchdog.applied_sequence) ? watchdog.applied_sequence : "—"}；仅状态计数，非 ACK。`;
    const attempted = numeric(metadata.attempted_at_unix) ? clock(metadata.attempted_at_unix) : "—";
    card.querySelector(".watchdog-last-check").textContent = `上次成功检测：${checked ? clock(metadata.checked_at_unix) : "—"} · 本次尝试：${attempted}。不自动轮询车端 HTTP。`;
  }

  function renderConnections() {
    const known = new Map(cars().map((car) => [car.id, car]));
    connectionRows().forEach((row) => { if (!known.has(row.id)) known.set(row.id, { id: row.id }); });
    if (!state.connectionSelectionInitialized && known.size) {
      state.connectionSelected = new Set(known.keys());
      state.connectionSelectionInitialized = true;
    }
    const selected = [...state.connectionSelected].filter((id) => known.has(id));
    const apiAvailable = connectionAvailable();
    const readDisabled = !state.online || !apiAvailable || state.connectionPending || !selected.length;
    const mutationDisabled = readDisabled || Boolean(busy()) || state.connectionUnknown;
    const rowMutationDisabled = !state.online || !apiAvailable || state.connectionPending || Boolean(busy()) || state.connectionUnknown;
    const selectedBusy = connectionRows().some((row) => selected.includes(row.id) && row.busy);
    $("connection-check").disabled = readDisabled;
    $("connection-connect").disabled = mutationDisabled || selectedBusy;
    $("connection-connect").title = busy() ? "运动任务期间不可变更连接" : selectedBusy ? "所选车辆正在处理连接，请等待逐车结果" : "仅检测并连接，配置不匹配时会等待明确操作";
    $("connection-select-all").disabled = !known.size;
    $("connection-clear").disabled = !selected.length;
    $("connection-selected-count").textContent = `连接目标 ${selected.length} 辆`;
    const processing = connectionRows().filter((row) => row.busy).length;
    $("connections-count").textContent = processing ? `${processing} 辆处理中` : `${known.size} 辆配置车辆`;
    $("connections-owned").textContent = `本页持有 ${connectionRows().filter((row) => row.owned).length} · 外部连接 ${connectionRows().filter((row) => row.phase === "external").length}`;
    const notice = $("connections-unavailable");
    notice.hidden = state.online && apiAvailable && !busy() && !state.connectionUnknown;
    if (!state.snapshot) notice.textContent = "正在读取连接管理能力；页面不会自动发送连接请求。";
    else if (!connections()) notice.textContent = "当前服务尚未提供网页连接管理，请更新并重启网页服务。现有运动控制与动捕显示仍可使用。";
    else if (!apiAvailable) notice.textContent = connections().error || "连接管理暂不可用；现有外部连接不受本页操作。";
    else if (!state.online) notice.textContent = "服务失联，不能确认连接阶段。不会自动重试连接，请等待恢复。";
    else if (busy()) notice.textContent = "运动任务执行中：连接、应用配置与关闭连接已锁定；“仅检测”和停止按钮仍可使用。";
    else if (state.connectionUnknown) notice.textContent = "上次连接操作的提交结果待确认，正在读取后台状态；不会自动重发。";
    const rows = new Map(connectionRows().map((row) => [row.id, row]));
    for (const [id, card] of state.connectionCards) {
      if (!known.has(id)) { card.remove(); state.connectionCards.delete(id); }
    }
    for (const [id, car] of known) {
      const row = rows.get(id) || { id, phase: "idle", message: "尚未检测车辆通信", busy: false, owned: false };
      const card = state.connectionCards.get(id) || createConnectionCard(car);
      card.querySelector("input").checked = state.connectionSelected.has(id);
      card.classList.toggle("selected", state.connectionSelected.has(id));
      card.classList.toggle("connection-cached", !state.online);
      const [label, style] = CONNECTION_PHASES[row.phase] || ["未知阶段", "idle"];
      const phase = card.querySelector(".connection-phase");
      phase.className = `connection-phase ${style}`;
      phase.textContent = state.online ? label : `上次：${label}`;
      card.dataset.phase = row.phase;
      card.querySelector(".connection-card-ip").textContent = car.ip || "IP 以车队配置为准";
      card.querySelector(".connection-message").textContent = row.message || (row.busy ? "后台正在处理，请等待状态更新" : label);
      card.querySelector(".connection-error").textContent = row.error || "";
      card.querySelector(".connection-error").hidden = !row.error;
      renderConnectionWatchdog(card, row);
      card.querySelector(".connection-config").hidden = row.phase !== "needs_config";
      card.querySelector(".connection-current").textContent = connectionRuntime(row.current);
      card.querySelector(".connection-target").textContent = connectionRuntime(row.target);
      card.querySelector(".connection-current-detail").textContent = connectionRuntime(row.current);
      card.querySelector(".connection-target-detail").textContent = connectionRuntime(row.target);
      card.querySelector(".connection-configure").disabled = rowMutationDisabled || Boolean(row.busy);
      const cancel = card.querySelector(".connection-cancel");
      cancel.hidden = (!row.owned && !row.busy) || row.phase === "external";
      cancel.textContent = row.owned ? "关闭本页连接" : "取消连接";
      cancel.disabled = rowMutationDisabled || row.phase === "stopping";
      cancel.title = "取消未完成的连接步骤或释放本页启动的服务，不发送停车命令";
      card.querySelector(".connection-owner").textContent = row.phase === "external" ? "终端 / 外部启动 · 仅查看" : row.owned ? "本页启动的连接" : "无本页持有的连接";
      card.querySelector(".connection-log-file").textContent = row.log_file ? `日志文件：${row.log_file}` : "暂无日志文件";
      card.querySelector(".connection-log-tail").textContent = row.log_tail || "暂无日志输出";
    }
  }

  async function connectionAction(action, explicitIds = null) {
    const ids = explicitIds || [...state.connectionSelected];
    if (!state.online || !state.csrf || !connectionAvailable() || state.connectionPending || !ids.length) return;
    if (action !== "check" && (busy() || state.connectionUnknown)) return;
    const rows = new Map(connectionRows().map((row) => [row.id, row]));
    if (action === "configure" && ids.some((id) => rows.get(id)?.phase !== "needs_config" || rows.get(id)?.busy)) return;
    if (action === "cancel" && ids.some((id) => {
      const row = rows.get(id);
      return !row || (!row.owned && !row.busy) || row.phase === "external" || row.phase === "stopping";
    })) return;
    if (action === "connect" && ids.some((id) => rows.get(id)?.busy)) return;
    state.connectionPending = true;
    if (action !== "check") state.connectionMutationTargets = new Set(ids);
    const verb = action === "cancel" && ids.every((id) => !rows.get(id)?.owned) ? "取消连接" : { check: "仅检测", connect: "检测并连接", configure: "应用配置并连接（会重启）", cancel: "关闭本页连接" }[action];
    connectionFeedback(`正在提交${verb}：${ids.join("、")}。请等待后台逐车进度。`);
    renderConnections(); renderControls();
    try {
      const response = await request("/api/connections", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: JSON.stringify({ action, cars: ids }) });
      if (response.connections && state.snapshot) state.snapshot.connections = response.connections;
      connectionFeedback(action === "configure" ? "已提交配置操作。服务会重新核对当前值与目标值；是否已重启请以逐车阶段与日志为准。" : `${verb}已提交，请查看逐车进度。${action === "cancel" ? "关闭通信连接不代表发送停车命令。" : "不会自动重试。"}`);
      log(`${verb}已提交：${ids.join("、")}。`, action === "configure" ? "warning" : "info");
    } catch (error) {
      state.connectionUnknown = action !== "check" && (!error.httpStatus || error.httpStatus >= 500);
      state.connectionUnknownSince = Date.now();
      if (!error.httpStatus) { state.online = false; state.connectionError = error.message; }
      connectionFeedback(`${verb}请求未确认：${error.message}。请查看逐车状态；不会自动重发。`, true);
      log(`${verb}请求未确认：${error.message}。`, "error");
    } finally {
      state.connectionPending = false;
      state.connectionMutationTargets.clear();
      render();
      poll();
    }
  }

  function mocapView(car) {
    const raw = car.mocap && typeof car.mocap === "object" ? car.mocap : {};
    const position = raw.position;
    const validPosition = position && [position.x, position.y, position.z].every(numeric);
    let status = Object.hasOwn(MOCAP_STATUS, raw.status) ? raw.status : "missing";
    if (status === "fresh" && !validPosition) status = "invalid";
    const serviceFresh = state.online && Date.now() - state.receivedAt <= 2000;
    const fresh = serviceFresh && status === "fresh";
    const frame = typeof raw.frame_id === "string" && raw.frame_id.trim() ? raw.frame_id : "";
    const age = numeric(raw.age_s) ? Math.max(0, raw.age_s) + Math.max(0, Date.now() - state.receivedAt) / 1000 : null;
    return {
      raw, status, fresh, frame, position: validPosition ? position : null, age,
      label: !serviceFresh && status !== "missing" ? "上次状态 · 待更新" : MOCAP_STATUS[status],
      color: !serviceFresh && status !== "missing" ? "stale" : status,
      plottable: Boolean(validPosition && frame && ["fresh", "stale"].includes(status)),
    };
  }

  function mocapAge(view) {
    return view.age === null ? "年龄未知" : `${view.age < 10 ? view.age.toFixed(2) : view.age.toFixed(1)} s 前`;
  }

  function clock(value) {
    let date;
    if (typeof value === "number") date = new Date(value < 1e12 ? value * 1000 : value);
    else date = value ? new Date(value) : new Date();
    return Number.isNaN(date.getTime()) ? String(value || "—") : date.toLocaleTimeString("zh-CN", { hour12: false });
  }

  function log(message, level = "info") {
    state.localEvents.unshift({ time: Date.now(), level, message, local: true });
    state.localEvents = state.localEvents.slice(0, 40);
    renderEvents();
  }

  function message(text, error = false) {
    $("action-message").textContent = text;
    $("action-message").classList.toggle("error", error);
    $("action-message").hidden = !text;
  }

  function selectCar(id, checked) {
    if (busy() || state.controlMode === "trajectory") return;
    const car = cars().find((entry) => entry.id === id);
    if (checked && (!state.online || !car || !isConnected(car))) return;
    if (checked) {
      state.selected.add(id);
      if (!state.individual.has(id)) state.individual.set(id, { v: $("uniform-v").value, w: $("uniform-w").value });
    } else state.selected.delete(id);
    render();
  }

  function commandDraft() {
    const duration = Number($("duration").value);
    const selected = cars().filter((car) => state.selected.has(car.id));
    if (!state.online || Date.now() - state.receivedAt > 2000) return { error: "控制服务未连接或状态已过期，启动已禁用。停止按钮仍可尝试发送。" };
    if (busy()) return { error: "任务期间显示只读参与清单；结束后恢复本页选车草稿。可随时停止。" };
    if (state.unknownOutcome) return { error: "上次启动结果尚未确认，已禁用重复启动。正在读取状态，可使用下方停止按钮。" };
    if (state.connectionUnknown) return { error: "上次连接操作结果待确认，正在读取后台状态；确认前不能启动运动。" };
    if (!selected.length) return { error: "请选择至少一辆连接正常的小车。" };
    const unconfirmedStops = Array.isArray(state.snapshot?.stop_unconfirmed) ? state.snapshot.stop_unconfirmed : [];
    const waitingForStop = selected.filter((car) => unconfirmedStops.includes(car.id));
    if (waitingForStop.length) return { error: `${waitingForStop.map((car) => car.id).join("、")} 上次里程计停车反馈未确认。请先再次停止相关车辆或使用“全车发送零速度”，满足反馈准则后再启动。` };
    const changing = connectionTargetsBusy(state.selected);
    if (changing.length) return { error: `${[...new Set(changing)].join("、")} 正在变更连接，请等待连接操作完成后再启动运动。` };
    const unavailable = selected.filter((car) => !isConnected(car));
    if (unavailable.length) return { error: `${unavailable.map((car) => car.id).join("、")} 连接状态不满足要求，请取消选择或等待恢复。` };
    if (selected.length !== state.selected.size) return { error: "选中的车辆已不在配置中，请清空后重新选择。" };
    if (!finite($("duration").value) || duration < 0.1 || duration > 60) return { error: "持续时间必须为 0.1–60 秒的有效数值。" };
    const commands = {};
    for (const car of selected) {
      const values = state.mode === "uniform"
        ? { v: $("uniform-v").value, w: $("uniform-w").value }
        : state.individual.get(car.id);
      if (!values || !finite(values.v) || !finite(values.w)) return { error: `${car.id} 的 v 和 w 都必须填写有限数值。` };
      commands[car.id] = { v: Number(values.v), w: Number(values.w) };
    }
    return { commands, duration, rate: 20 };
  }

  function createCard(car) {
    const card = document.createElement("article");
    card.className = "car-card";
    card.setAttribute("role", "listitem");
    card.dataset.carId = car.id;
    card.innerHTML = '<div class="card-top"><label class="car-select"><input class="card-checkbox" type="checkbox"><strong></strong></label><span class="card-state"><i class="dot"></i><span></span></span></div><div class="card-ip"></div><div class="card-measurements"><div><div class="measurement-label">实测 v</div><div class="measurement-value value-v"></div></div><div><div class="measurement-label">实测 w</div><div class="measurement-value value-w"></div></div></div><div class="card-bottom"><span class="card-domain"></span><span class="card-age"></span></div><div class="card-warning" hidden></div><section class="card-mocap" aria-label="独立动捕观测"><div class="card-mocap-heading"><span>动捕</span><span class="mocap-status missing"></span></div><div class="mocap-coordinates"><div><span>X · m</span><strong class="mocap-x">—</strong></div><div><span>Y · m</span><strong class="mocap-y">—</strong></div><div><span>Z · m</span><strong class="mocap-z">—</strong></div></div><div class="mocap-yaw-row"><span>原始刚体航向 <strong class="mocap-yaw">—</strong></span><span class="mocap-age"></span></div><div class="mocap-meta-row"><span class="mocap-binding"></span><span class="mocap-publishers"></span></div><div class="mocap-frame-line"></div><details class="mocap-source"><summary>动捕来源</summary><div class="mocap-topic"></div><div class="mocap-source-domain"></div><div class="mocap-error" hidden></div></details></section>';
    const checkbox = card.querySelector("input");
    checkbox.setAttribute("aria-label", `选择 ${car.id}`);
    checkbox.addEventListener("change", () => selectCar(car.id, checkbox.checked));
    card.querySelector("strong").textContent = car.id;
    const csvTag = document.createElement("span"); csvTag.className = "csv-card-tag"; csvTag.textContent = "CSV 参与车辆"; csvTag.hidden = true;
    card.querySelector(".card-ip").after(csvTag);
    $("car-grid").append(card);
    state.cards.set(car.id, card);
    return card;
  }

  function ageText(car) {
    if (typeof car.age_s !== "number" || !Number.isFinite(car.age_s)) return "无里程计数据";
    const age = Math.max(0, car.age_s) + (state.online ? 0 : (Date.now() - state.receivedAt) / 1000);
    return `${state.online ? "数据" : "上次数据"} ${age < 10 ? age.toFixed(1) : Math.floor(age)} s 前`;
  }

  function renderCars() {
    const query = $("car-search").value.trim().toLowerCase();
    const onlineOnly = $("online-only").checked;
    let visible = 0;
    const known = new Set(cars().map((car) => car.id));
    for (const [id, card] of state.cards) {
      if (!known.has(id)) { card.remove(); state.cards.delete(id); }
    }
    for (const car of cars()) {
      const card = state.cards.get(car.id) || createCard(car);
      const selected = activeJob() ? displayedIds().has(car.id) : state.selected.has(car.id);
      const eligible = state.online && isConnected(car);
      const [label, color] = STATUS[car.status] || ["未知状态", "offline"];
      const matches = `${car.id} ${car.name || ""} ${car.ip || ""}`.toLowerCase().includes(query);
      card.hidden = !matches || (onlineOnly && !eligible);
      if (!card.hidden) visible++;
      card.classList.toggle("selected", selected);
      card.classList.toggle("unavailable", !isConnected(car));
      card.classList.toggle("stale-snapshot", !state.online);
      card.querySelector("input").checked = selected;
      card.querySelector("input").disabled = Boolean(busy()) || state.controlMode === "trajectory" || (!eligible && !selected);
      card.querySelector(".csv-card-tag").hidden = state.controlMode !== "trajectory" || !state.trajectorySelected.has(car.id);
      card.querySelector(".card-state").className = `card-state ${state.online ? color : "offline"}`;
      card.querySelector(".card-state span").textContent = state.online ? label : `上次：${label}`;
      card.querySelector(".card-ip").textContent = car.ip || "未配置 IP";
      for (const axis of ["v", "w"]) {
        const value = card.querySelector(`.value-${axis}`);
        value.textContent = number(car[`measured_${axis}`]);
        const unit = document.createElement("small");
        unit.textContent = axis === "v" ? "m/s" : "rad/s";
        value.append(unit);
      }
      card.querySelector(".card-domain").textContent = `DOMAIN ${car.domain_id ?? "—"}`;
      card.querySelector(".card-age").textContent = ageText(car);
      card.title = `${car.id} · 速度订阅者 ${car.cmd_subscribers ?? "—"} · 速度发布者 ${car.cmd_publishers ?? "—"}${car.error ? ` · ${car.error}` : ""}`;
      const detail = car.status === "discovered"
        ? `速度订阅 ${car.cmd_subscribers ?? 0} · ${typeof car.age_s === "number" ? "已有里程计数据" : "暂无里程计"}` : "";
      const stopWarning = Array.isArray(state.snapshot?.stop_unconfirmed) && state.snapshot.stop_unconfirmed.includes(car.id) ? "上次里程计停车反馈未确认，请再次停止" : "";
      const warning = [car.error || (car.cmd_publishers > 1 ? "检测到多个速度发布者" : detail), stopWarning].filter(Boolean).join("；");
      card.querySelector(".card-warning").textContent = warning;
      card.querySelector(".card-warning").hidden = !warning;
      renderCardMocap(card, car);
    }
    $("visible-count").textContent = String(visible);
    $("fleet-empty").hidden = visible > 0;
    if (!visible) {
      const empty = $("fleet-empty");
      empty.querySelector("strong").textContent = !state.snapshot ? "等待控制服务提供车队状态" : "没有符合条件的车辆";
      empty.querySelector("span").textContent = !state.snapshot ? "连接建立后将显示已配置车辆" : "可调整搜索内容或取消“只看在线”";
    }
  }

  function renderCardMocap(card, car) {
    const view = mocapView(car);
    const section = card.querySelector(".card-mocap");
    section.dataset.status = view.color;
    section.classList.toggle("cached", !view.fresh);
    const badge = card.querySelector(".mocap-status");
    badge.className = `mocap-status ${view.color}`;
    badge.textContent = view.label;
    ["x", "y", "z"].forEach((axis) => {
      card.querySelector(`.mocap-${axis}`).textContent = view.position ? number(view.position[axis]) : "—";
    });
    card.querySelector(".mocap-yaw").textContent = numeric(view.raw.yaw_rad) ? `${number(view.raw.yaw_rad * 180 / Math.PI, 1)}°` : "—";
    card.querySelector(".mocap-age").textContent = mocapAge(view);
    const binding = card.querySelector(".mocap-binding");
    binding.textContent = view.raw.binding_confirmed ? "车号绑定已确认" : "车号绑定待确认";
    binding.classList.toggle("confirmed", view.raw.binding_confirmed === true);
    const publishers = card.querySelector(".mocap-publishers");
    publishers.textContent = `发布者 ${numeric(view.raw.publishers) ? view.raw.publishers : "—"}${view.raw.publishers > 1 ? " · 多个来源" : ""}`;
    publishers.classList.toggle("multiple", view.raw.publishers > 1);
    const frame = card.querySelector(".mocap-frame-line");
    frame.textContent = `frame: ${view.frame || "未提供 · 不绘图"}`;
    frame.title = frame.textContent;
    card.querySelector(".mocap-topic").textContent = view.raw.topic || "后端尚未提供动捕话题";
    card.querySelector(".mocap-source-domain").textContent = `ROS 域 ${view.raw.domain_id ?? "—"} · ${view.fresh ? "近期接收，不代表有效跟踪" : "显示最近收到的数据，不代表当前位置"}`;
    card.querySelector(".mocap-error").textContent = view.raw.error || "";
    card.querySelector(".mocap-error").hidden = !view.raw.error;
  }

  function svgNode(tag, attributes = {}, text = null) {
    const element = document.createElementNS("http://www.w3.org/2000/svg", tag);
    Object.entries(attributes).forEach(([name, value]) => element.setAttribute(name, value));
    if (text !== null) element.textContent = text;
    return element;
  }

  function mapExtent(points, frame) {
    const width = 806, height = 344;
    const saved = state.mocapBounds.get(frame);
    if (saved && points.every(({ view }) => {
      const { x, y } = view.position;
      return x >= saved.xmin + saved.spanX * 0.07 && x <= saved.xmax - saved.spanX * 0.07
        && y >= saved.ymin + saved.spanY * 0.07 && y <= saved.ymax - saved.spanY * 0.07;
    })) return saved;
    const xs = points.map(({ view }) => view.position.x), ys = points.map(({ view }) => view.position.y);
    let xmin = xs.length ? Math.min(...xs) : -1.5, xmax = xs.length ? Math.max(...xs) : 1.5;
    let ymin = ys.length ? Math.min(...ys) : -1.5, ymax = ys.length ? Math.max(...ys) : 1.5;
    if (saved) { xmin = Math.min(xmin, saved.xmin); xmax = Math.max(xmax, saved.xmax); ymin = Math.min(ymin, saved.ymin); ymax = Math.max(ymax, saved.ymax); }
    const centerX = xmin / 2 + xmax / 2, centerY = ymin / 2 + ymax / 2;
    const scale = Math.min(width / Math.max(3, (xmax - xmin) * 1.3), height / Math.max(3, (ymax - ymin) * 1.3));
    if (!numeric(scale) || scale <= 0) return null;
    const spanX = width / scale, spanY = height / scale;
    const extent = { xmin: centerX - spanX / 2, xmax: centerX + spanX / 2, ymin: centerY - spanY / 2, ymax: centerY + spanY / 2, spanX, spanY, scale };
    if (!Object.values(extent).every(numeric)) return null;
    state.mocapBounds.set(frame, extent);
    return extent;
  }

  function renderMocapMap() {
    const views = cars().map((car) => ({ car, view: mocapView(car) }));
    const preview = state.controlMode === "trajectory" ? trajectoryCars() : [];
    const previewFrame = trajectoryFrameInfo();
    if (preview.length && state.trajectoryPreviewFrameKey !== previewFrame.key) {
      state.trajectoryPreviewFrameKey = previewFrame.key; state.mocapFrame = previewFrame.key;
    }
    const frames = [...new Set([...views.map(({ view }) => view.frame).filter(Boolean), ...(preview.length ? [previewFrame.key] : [])])].sort();
    if (state.mocapFrame === null && frames.length) state.mocapFrame = frames[0];
    const chosen = state.mocapFrame || "";
    const availableFrames = frames.includes(chosen) || !chosen ? frames : [chosen, ...frames];
    const key = JSON.stringify(availableFrames);
    if (key !== state.mocapFramesKey) {
      state.mocapFramesKey = key;
      $("mocap-frame").replaceChildren();
      if (!availableFrames.length) {
        const option = document.createElement("option"); option.value = ""; option.textContent = "暂无坐标系"; $("mocap-frame").append(option);
      } else availableFrames.forEach((frame) => {
        const option = document.createElement("option"); option.value = frame; option.textContent = frame === "@csv-world-preview-unverified" ? "CSV 独立预览 · 坐标系待核对" : frame; $("mocap-frame").append(option);
      });
    }
    $("mocap-frame").value = chosen;
    $("mocap-frame").disabled = !availableFrames.length;
    const inFrame = views.filter(({ view }) => chosen && view.frame === chosen);
    const points = inFrame.filter(({ view }) => view.plottable);
    const paths = chosen === previewFrame.key ? preview.map((car) => {
      const valid = (Array.isArray(car.points) ? car.points : []).filter((point) => numeric(point.x) && numeric(point.y));
      const stride = Math.max(1, Math.ceil(valid.length / 800));
      const sampled = valid.filter((_, index) => index % stride === 0);
      if (valid.length && sampled[sampled.length - 1] !== valid[valid.length - 1]) sampled.push(valid[valid.length - 1]);
      return { car, points: sampled };
    }).filter((path) => path.points.length) : [];
    const fresh = views.filter(({ view }) => view.fresh).length;
    $("mocap-fresh-count").textContent = `${fresh} 份新鲜数据`;
    $("mocap-map-frame").textContent = chosen === "@csv-world-preview-unverified" ? "CSV world 独立预览 · XY 等比例 / 米 · 坐标系待核对" : chosen ? `frame: ${chosen} · XY 等比例 / 米` : "等待明确的动捕坐标系";
    $("mocap-map-live").textContent = !state.online ? "服务未连接 · 上次位置" : points.some(({ view }) => view.fresh) ? "近期数据 · 非跟踪质量判定" : "暂无新鲜位置";
    $("mocap-map-live").classList.toggle("cached", !state.online || !points.some(({ view }) => view.fresh));
    $("mocap-frame-count").textContent = `${inFrame.length} 辆`;
    $("mocap-unplotted").textContent = `当前 frame 已绘 ${points.length} / ${inFrame.length} 辆。${views.filter(({ view }) => !view.frame).length} 辆未提供 frame；无坐标、无效或异常数据不绘图。`;
    $("mocap-fit").disabled = !points.length && !paths.length;
    const list = $("mocap-frame-cars");
    list.replaceChildren();
    if (!inFrame.length) {
      const empty = document.createElement("p"); empty.className = "small-empty"; empty.textContent = "此 frame 暂无车辆观测"; list.append(empty);
    }
    inFrame.forEach(({ car, view }) => {
      const row = document.createElement("div"); row.className = `mocap-observation${displayedIds().has(car.id) ? " selected" : ""}`; row.dataset.carId = car.id;
      const top = document.createElement("div"); top.className = "mocap-observation-top";
      const name = document.createElement("strong"); name.textContent = car.id;
      const badge = document.createElement("span"); badge.className = `mocap-status ${view.color}`; badge.textContent = view.label;
      top.append(name, badge);
      const position = document.createElement("div"); position.className = "mocap-observation-position";
      position.textContent = view.position ? `X ${number(view.position.x)} · Y ${number(view.position.y)} m` : "尚无合法位置坐标";
      const meta = document.createElement("div"); meta.className = "mocap-observation-meta";
      meta.textContent = `${mocapAge(view)} · ${view.raw.binding_confirmed ? "绑定已确认" : "绑定待确认"}${view.raw.publishers > 1 ? ` · ${view.raw.publishers} 个来源` : ""}`;
      row.append(top, position, meta); list.append(row);
    });
    $("mocap-grid").replaceChildren(); $("mocap-points").replaceChildren(); $("trajectory-paths").replaceChildren();
    const boundsPoints = [...points, ...paths.flatMap((path) => path.points.map((point) => ({ view: { position: point } })))];
    const extent = mapExtent(boundsPoints, chosen);
    $("mocap-map-empty").hidden = boundsPoints.length > 0 && Boolean(extent);
    const legend = $("trajectory-map-legend"); legend.replaceChildren(); legend.hidden = !preview.length;
    if (preview.length) {
      const description = document.createElement("span"); description.className = "trajectory-map-caption";
      description.textContent = chosen === previewFrame.key ? `${previewFrame.known ? `CSV frame: ${previewFrame.label}` : "CSV world 独立预览，未叠加动捕，坐标系待核对"} · 图为采样预览，执行采用完整文件` : `CSV 路线坐标系：${previewFrame.label}；切换后查看，当前 frame 不叠加`;
      legend.append(description);
      preview.forEach((car) => {
        const item = document.createElement("span"); const marker = document.createElement("i"); marker.style.backgroundColor = trajectoryColor(car.car_id);
        item.append(marker, document.createTextNode(`${car.car_id}${state.trajectorySelected.has(car.car_id) ? " · 已选" : " · 未选"}`)); legend.append(item);
      });
    }
    if (!extent) {
      $("mocap-map-empty").querySelector("strong").textContent = "坐标跨度超出绘图范围";
      return;
    }
    $("mocap-map-empty").querySelector("strong").textContent = "尚无可绘制的动捕位置";
    const px = (x) => 62 + (x - extent.xmin) * extent.scale;
    const py = (y) => 24 + (extent.ymax - y) * extent.scale;
    const rawStep = extent.spanY / 5;
    const base = 10 ** Math.floor(Math.log10(rawStep));
    const ratio = rawStep / base;
    const step = (ratio <= 1 ? 1 : ratio <= 2 ? 2 : ratio <= 5 ? 5 : 10) * base;
    const grid = $("mocap-grid");
    grid.append(svgNode("rect", { x: 62, y: 24, width: 806, height: 344, class: "mocap-plot-background" }));
    const decimals = Math.min(6, Math.max(0, -Math.floor(Math.log10(step))));
    for (let x = Math.ceil(extent.xmin / step) * step, i = 0; x <= extent.xmax && i < 30; x += step, i++) {
      const zero = Math.abs(x) < step * 0.001;
      grid.append(svgNode("line", { x1: px(x), x2: px(x), y1: 24, y2: 368, class: zero ? "mocap-axis-zero" : "mocap-grid-line" }));
      grid.append(svgNode("text", { x: px(x), y: 389, class: "mocap-tick", "text-anchor": "middle" }, zero ? "0" : x.toFixed(decimals)));
    }
    for (let y = Math.ceil(extent.ymin / step) * step, i = 0; y <= extent.ymax && i < 20; y += step, i++) {
      const zero = Math.abs(y) < step * 0.001;
      grid.append(svgNode("line", { x1: 62, x2: 868, y1: py(y), y2: py(y), class: zero ? "mocap-axis-zero" : "mocap-grid-line" }));
      grid.append(svgNode("text", { x: 51, y: py(y) + 4, class: "mocap-tick", "text-anchor": "end" }, zero ? "0" : y.toFixed(decimals)));
    }
    grid.append(svgNode("text", { x: 870, y: 411, class: "mocap-axis-label", "text-anchor": "end" }, "X / m →"));
    grid.append(svgNode("text", { x: 16, y: 15, class: "mocap-axis-label" }, "Y / m ↑"));
    paths.forEach(({ car, points: route }) => {
      const selected = state.trajectorySelected.has(car.car_id);
      const group = svgNode("g", { class: `trajectory-map-route${selected ? " selected" : ""}`, "data-car-id": car.car_id });
      const color = trajectoryColor(car.car_id);
      group.append(svgNode("title", {}, `${car.car_id} · CSV world 路线 · ${car.duration} s · ${car.sample_count} 点`));
      group.append(svgNode("polyline", { points: route.map((point) => `${px(point.x)},${py(point.y)}`).join(" "), stroke: color, class: "trajectory-map-line" }));
      const start = route[0], end = route[route.length - 1];
      group.append(svgNode("circle", { cx: px(start.x), cy: py(start.y), r: 4, fill: color, class: "trajectory-map-start" }));
      group.append(svgNode("circle", { cx: px(end.x), cy: py(end.y), r: 6, stroke: color, class: "trajectory-map-end" }));
      group.append(svgNode("text", { x: Math.min(830, px(end.x) + 10), y: Math.max(40, py(end.y) - 9), fill: color, class: "trajectory-map-label" }, `${car.car_id} CSV`));
      $("trajectory-paths").append(group);
    });
    points.sort((a, b) => Number(displayedIds().has(a.car.id)) - Number(displayedIds().has(b.car.id)));
    points.forEach(({ car, view }, index) => {
      const selected = displayedIds().has(car.id);
      const group = svgNode("g", { class: `mocap-point ${view.fresh ? "fresh" : "stale"}${selected ? " selected" : ""}`, "data-car-id": car.id, "data-status": view.fresh ? "fresh" : "stale", transform: `translate(${px(view.position.x)},${py(view.position.y)})` });
      group.append(svgNode("title", {}, `${car.id} · ${view.label}\nX ${number(view.position.x)} m · Y ${number(view.position.y)} m · Z ${number(view.position.z)} m\n${mocapAge(view)} · ${view.raw.binding_confirmed ? "车号绑定已确认" : "车号绑定待确认"}\n原始刚体 +X 未校准车头`));
      if (selected) group.append(svgNode("circle", { r: 13, class: "mocap-selection-ring" }));
      group.append(svgNode("circle", { r: 6, class: "mocap-position-dot" }));
      if (numeric(view.raw.yaw_rad)) {
        const angle = -view.raw.yaw_rad * 180 / Math.PI;
        const arrow = svgNode("g", { transform: `rotate(${angle})`, class: "mocap-body-axis" });
        arrow.append(svgNode("path", { d: "M9 0H29M23 -4L29 0L23 4" })); group.append(arrow);
      }
      group.append(svgNode("text", { x: 11, y: index % 2 ? 24 : -13, class: "mocap-point-label" }, car.id));
      $("mocap-points").append(group);
    });
  }

  function renderTargets() {
    const selected = cars().filter((car) => displayedIds().has(car.id));
    const targetKey = selected.map((car) => `${car.id}:${car.status}`).join("|");
    if ($("selected-targets").dataset.key !== targetKey) {
      const target = $("selected-targets");
      target.replaceChildren();
      target.dataset.key = targetKey;
      if (!selected.length) {
        const empty = document.createElement("span");
        empty.className = "targets-placeholder";
        empty.textContent = "从左侧选择本次需要控制的车辆";
        target.append(empty);
      } else selected.forEach((car) => {
        const chip = document.createElement("span");
        chip.className = `target-chip${isConnected(car) ? "" : " invalid"}`;
        chip.textContent = car.id;
        target.append(chip);
      });
    }
    $("target-count").textContent = `${selected.length} 辆`;
    const rowKey = selected.map((car) => car.id).join("|");
    if (rowKey !== state.rowsKey) {
      state.rowsKey = rowKey;
      $("individual-rows").replaceChildren();
      selected.forEach((car) => {
        const row = document.createElement("tr");
        const name = document.createElement("td");
        name.textContent = car.id;
        row.append(name);
        if (!state.individual.has(car.id)) state.individual.set(car.id, { v: $("uniform-v").value, w: $("uniform-w").value });
        ["v", "w"].forEach((axis) => {
          const cell = document.createElement("td");
          const input = document.createElement("input");
          input.type = "number";
          input.step = "0.01";
          input.inputMode = "decimal";
          input.value = state.individual.get(car.id)[axis];
          input.setAttribute("aria-label", `${car.id} ${axis}，${axis === "v" ? "米每秒" : "弧度每秒"}`);
          input.addEventListener("input", () => {
            state.individual.get(car.id)[axis] = input.value;
            renderControls();
          });
          cell.append(input);
          row.append(cell);
        });
        $("individual-rows").append(row);
      });
    }
    $("individual-empty").hidden = selected.length > 0;
    $("individual-table").hidden = !selected.length;
  }

  function renderControls() {
    const locked = Boolean(busy());
    const csvMode = state.controlMode === "trajectory";
    document.body.classList.toggle("busy", locked);
    $("command-fields").disabled = locked;
    $("command-fields").hidden = csvMode;
    $("velocity-participants").hidden = csvMode;
    $("trajectory-controls").hidden = !csvMode;
    for (const mode of ["velocity", "trajectory"]) {
      $("control-" + mode).disabled = locked;
      $("control-" + mode).classList.toggle("active", state.controlMode === mode);
      $("control-" + mode).setAttribute("aria-pressed", String(state.controlMode === mode));
    }
    $("vehicle-selection-hint").textContent = csvMode ? "CSV 参与车辆在右侧独立选择；此处保留速度草稿" : "仅连接正常的车辆可启动";
    $("select-online").disabled = locked || csvMode || !state.online || !cars().some(isConnected);
    $("clear-selection").disabled = locked || csvMode || !state.selected.size;
    $("configuration-state").textContent = locked ? "当前任务 · 只读" : (csvMode ? state.trajectorySelected.size : state.selected.size) ? "待启动" : "待配置";
    $("show-job-results").hidden = !job();
    for (const mode of ["uniform", "individual"]) {
      $("mode-" + mode).classList.toggle("active", state.mode === mode);
      $("mode-" + mode).setAttribute("aria-pressed", String(state.mode === mode));
      $(mode + "-controls").hidden = state.mode !== mode;
    }
    const current = activeJob();
    const selected = cars().filter((car) => displayedIds().has(car.id));
    const duration = current ? String(current.duration) : csvMode ? String(Math.max(0, ...trajectoryCars().filter((car) => state.trajectorySelected.has(car.car_id)).map((car) => car.duration || 0))) : $("duration").value;
    $("summary-title").textContent = current ? `当前任务 · ${selected.length} 辆参与车辆` : csvMode ? state.trajectory ? `CSV 轨迹 · ${state.trajectorySelected.size} 辆参与车辆` : "尚未导入 CSV" : selected.length ? `向 ${selected.length} 辆车发送${state.mode === "uniform" ? "统一" : "独立"}速度命令` : "尚未选择车辆";
    const lines = [];
    if (current?.kind === "trajectory") {
      lines.push(current.trajectory_filename || "CSV 轨迹", (current.cars || []).join(" · "), "world 坐标 · 共同计时 · 各车按自己的结束时间停车");
    } else if (!current && csvMode) {
      if (state.trajectory) lines.push(state.trajectory.filename, [...state.trajectorySelected].join(" · ") || "尚未选择 CSV 参与车辆", "world 坐标 · 共同计时 · 开始时服务端复核");
      else lines.push("文件导入后可预览与检查状态，点击开始才会提交任务；检查不是开始条件。");
    } else if (selected.length) {
      if (current) {
        lines.push(selected.map((car) => car.id).join(" · "));
        selected.forEach((car) => {
          const value = current.commands?.[car.id];
          if (value) lines.push(`${car.id}   v ${signed(value.v)} m/s   w ${signed(value.w)} rad/s`);
        });
        if (current.kind === "stop_all") lines.push("向全部配置车辆发送零速度");
      } else if (state.mode === "uniform") {
        const v = $("uniform-v").value, w = $("uniform-w").value;
        lines.push(selected.map((car) => car.id).join(" · "));
        lines.push(finite(v) && finite(w) ? `v ${signed(Number(v))} m/s · w ${signed(Number(w))} rad/s` : "请填写有效的 v 和 w 数值");
      } else selected.forEach((car) => {
        const value = state.individual.get(car.id);
        lines.push(`${car.id}   v ${finite(value?.v) ? signed(Number(value.v)) : "—"}   w ${finite(value?.w) ? signed(Number(value.w)) : "—"}`);
      });
    } else lines.push("配置完成后，将在此显示待发送的命令。");
    $("summary-commands").textContent = lines.join("\n");
    $("summary-commands").style.whiteSpace = "pre-line";
    $("summary-duration").textContent = `持续 ${finite(duration) ? Number(duration) : "—"} 秒`;
    const draft = csvMode ? trajectoryDraft() : commandDraft();
    $("run-button").disabled = Boolean(draft.error) || !state.csrf;
    $("run-button").querySelector("span").textContent = state.runPending ? "正在提交任务…" : locked ? current?.kind === "trajectory" ? "CSV 轨迹执行中" : "任务执行中" : csvMode ? "开始所选 CSV 轨迹" : "启动本次任务";
    $("run-hint").textContent = draft.error || (csvMode ? "可直接提交开始；检查仅提示风险，实际执行由服务端校验可用输入。" : "请核对摘要；点击后立即执行本次任务。");
    $("run-hint").classList.toggle("warning", Boolean(draft.error) && state.selected.size > 0 && !locked);
    const retryStop = !locked && Array.isArray(job()?.cars) && job().cars.length > 0;
    $("stop-job").disabled = !state.csrf || state.stopPending || !(locked || state.unknownOutcome || retryStop);
    $("stop-job").classList.toggle("retry", retryStop);
    $("stop-job").querySelector("span").textContent = state.stopPending ? "正在提交停止…" : retryStop ? "再次停止本次车辆" : "停止本次";
    $("stop-all").disabled = !state.csrf || state.stopPending;
    renderTrajectoryControls();
  }

  function renderJob() {
    const current = job();
    $("job-panel").hidden = !current;
    if (current) {
      $("job-status").textContent = current.kind === "stop_all" && current.status === "stopping"
        ? "全车停车中 · 等待反馈" : jobLabel(current);
      $("job-panel").classList.toggle("stop-unconfirmed", current.status === "unconfirmed" || ["completed", "stopped"].includes(current.status) && stopResults(current).some((result) => result.status !== "confirmed"));
      const remaining = typeof current.remaining_s === "number" ? Math.max(0, current.remaining_s) : null;
      $("job-remaining").replaceChildren(document.createTextNode(remaining === null ? "— " : `${remaining.toFixed(1)} `));
      const unit = document.createElement("small"); unit.textContent = "秒"; $("job-remaining").append(unit);
      const progress = current.duration > 0 && remaining !== null ? Math.max(0, Math.min(100, (1 - remaining / current.duration) * 100)) : 0;
      $("job-progress").style.width = `${progress}%`;
      $("job-detail").textContent = `${current.cars?.length || 0} 辆车 · ${current.duration ?? "—"} 秒`;
      $("job-id").textContent = `#${String(current.id).slice(0, 8)}`;
      $("job-error").textContent = current.error || "";
      $("job-error").hidden = !current.error;
    }
    $("trajectory-job-info").hidden = current?.kind !== "trajectory";
    if (current?.kind === "trajectory") {
      $("trajectory-job-file").textContent = current.trajectory_filename || "CSV 轨迹任务";
      $("trajectory-job-progress").replaceChildren();
      (current.cars || []).forEach((id) => {
        const progress = current.trajectory_progress?.[id];
        const row = document.createElement("div"); row.className = "trajectory-progress-row"; row.dataset.carId = id;
        const name = document.createElement("strong"); name.textContent = id;
        const detail = document.createElement("div"); detail.className = "trajectory-progress-detail";
        const text = document.createElement("span"); text.className = "trajectory-progress-time";
        const status = { waiting: "等待共同计时", running: "执行中", completed: "已到结束时间", stopped: "已停止执行" }[progress?.status] || "等待进度";
        text.textContent = `${status} · ${numeric(progress?.elapsed_s) ? progress.elapsed_s.toFixed(1) : "—"} / ${numeric(progress?.duration) ? progress.duration.toFixed(1) : "—"} s`;
        const mode = ["mocap", "estimated", "pending", "unavailable"].includes(progress?.localization_mode) ? progress.localization_mode : "unreported";
        const executing = state.online && ["starting", "running"].includes(current.status) && !["completed", "stopped"].includes(progress?.status);
        const gap = numeric(progress?.localization_gap_s) ? `${Math.max(0, progress.localization_gap_s).toFixed(1)} 秒` : "时长未报告";
        const localization = document.createElement("span");
        localization.className = `trajectory-localization ${mode}${state.online ? "" : " cached"}`;
        localization.dataset.localizationMode = mode;
        const labels = {
          mocap: "动捕跟踪", estimated: `动捕中断·推算${executing ? "继续" : ""}（${gap}）`,
          pending: "等待定位输入", unavailable: "定位输入不可用", unreported: "定位状态未报告",
        };
        const prefix = !state.online ? "上次状态 · " : executing ? "" : current.status === "stopping" ? "停车前 · " : "结束时 · ";
        localization.textContent = `${prefix}${labels[mode]}`;
        localization.title = mode === "estimated" ? "根据最后有效动捕位姿和已发送速度推算，并非当前动捕实测。" : "轨迹控制使用的定位输入；独立动捕状态仍反映实际接收数据。";
        detail.append(text, localization); row.append(name, detail); $("trajectory-job-progress").append(row);
      });
    }
    renderStopResults(current);
    const active = Boolean(busy());
    const needsConfirmation = current && !active && (current.status === "unconfirmed" || stopResults(current).some((result) => result.status !== "confirmed"));
    $("dock-indicator").className = `dock-indicator ${!state.online ? "offline" : active ? "running" : needsConfirmation ? "warning" : "ready"}`;
    $("dock-title").textContent = !state.online ? "服务连接中断 · 任务状态待确认" : state.runPending ? "正在提交任务" : current ? jobLabel(current) : "控制服务就绪";
    $("dock-description").textContent = !state.online ? "保留停止操作，页面不会自动重发运动任务" : current?.status === "stopping" ? "已进入停车流程，正在等待逐车里程计反馈" : active ? "停止将向本次参与车辆发送零速度" : needsConfirmation ? "逐车结果有未确认项；仍可再次停止本次车辆" : current ? "停车结果依据里程计反馈，并非固件 ACK" : "仅在点击启动后发送运动命令";
  }

  function renderEvents() {
    const events = [...state.localEvents, ...(state.snapshot?.events || [])];
    events.sort((a, b) => {
      const time = (event) => typeof event.time === "number" ? (event.time < 1e12 ? event.time * 1000 : event.time) : Date.parse(event.time) || 0;
      return time(b) - time(a);
    });
    const unique = Array.from(new Map(events.map((event) => [`${event.time}|${event.message}`, event])).values()).slice(0, 60);
    const key = JSON.stringify(unique);
    if (key === state.eventKey) return;
    state.eventKey = key;
    $("event-count").textContent = `${unique.length} 条记录`;
    if (!unique.length) return;
    $("event-list").replaceChildren();
    unique.forEach((event) => {
      const row = document.createElement("div"); row.className = "event-row";
      const time = document.createElement("span"); time.className = "event-time"; time.textContent = clock(event.time);
      const level = document.createElement("span");
      const kind = ["error", "warning", "success"].includes(event.level) ? event.level : "info";
      level.className = `event-level ${kind}`;
      level.textContent = { error: "异常", warning: "提示", success: "完成", info: "记录" }[kind];
      const text = document.createElement("span"); text.className = "event-message"; text.textContent = event.message;
      row.append(time, level, text); $("event-list").append(row);
    });
  }

  function render() {
    $("demo-banner").hidden = state.snapshot?.mode !== "demo";
    $("service-pill").className = `status-pill ${state.online ? "" : state.connectionError ? "offline" : "pending"}`;
    $("service-pill").querySelector("span").textContent = state.online ? "控制服务在线" : state.connectionError ? "控制服务失联" : "等待控制服务";
    $("connection-banner").hidden = state.online;
    $("connection-banner").textContent = state.receivedAt
      ? `控制服务暂不可达。最后成功更新 ${clock(state.receivedAt)}；下方为上次状态，不能确认车辆或任务当前状态。`
      : state.connectionError ? `无法连接本机控制服务：${state.connectionError}。启动已禁用，正在重新读取状态。` : "正在连接本机控制服务。建立连接前不会发送任何运动命令。";
    $("last-update").textContent = state.receivedAt ? `${state.online ? "更新于" : "最后更新"} ${clock(state.receivedAt)}` : "等待首次状态";
    $("total-count").textContent = state.snapshot ? String(cars().length) : "—";
    const online = cars().filter(isConnected).length;
    $("connected-count").textContent = state.online ? String(online) : "—";
    $("connected-label").textContent = state.snapshot?.mode === "demo" ? "模拟连接正常" : "连接正常";
    $("connection-note").textContent = state.online ? `${cars().length - online} 辆待连接` : "状态未确认";
    $("selected-count").textContent = String(displayedIds().size);
    $("selection-note").textContent = activeJob() ? "当前任务参与" : state.selected.size ? "明确选中" : "尚未选择";
    renderConnections(); renderCars(); renderMocapMap(); renderTargets(); renderControls(); renderJob(); renderEvents();
  }

  async function request(path, options = {}, timeoutMs = 4000) {
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), timeoutMs);
    try {
      const response = await fetch(path, { ...options, signal: controller.signal, credentials: "same-origin", cache: "no-store" });
      let data;
      try { data = await response.json(); } catch (_) { throw new Error("服务返回了无法读取的响应"); }
      if (!response.ok) {
        const error = new Error(data.error || `请求失败（${response.status}）`);
        error.httpStatus = response.status;
        throw error;
      }
      return data;
    } finally { clearTimeout(timer); }
  }

  async function poll() {
    if (state.polling) return;
    clearTimeout(state.timer);
    state.polling = true;
    const pollStarted = Date.now();
    try {
      const snapshot = await request("/api/state");
      if (!Array.isArray(snapshot.cars) || !["live", "demo"].includes(snapshot.mode) || !snapshot.csrf_token) throw new Error("控制服务状态格式无效");
      const wasOffline = !state.online;
      state.snapshot = snapshot;
      state.csrf = snapshot.csrf_token;
      state.online = true;
      state.connectionError = "";
      state.receivedAt = Date.now();
      if (snapshot.job?.kind === "trajectory" && BUSY.has(snapshot.job.status)) state.controlMode = "trajectory";
      if (!state.runPending && pollStarted >= state.unknownSince) state.unknownOutcome = false;
      if (!state.connectionPending && pollStarted >= state.connectionUnknownSince) state.connectionUnknown = false;
      const jobKey = snapshot.job ? `${snapshot.job.id}:${snapshot.job.status}` : "";
      if (jobKey && jobKey !== state.lastJobKey) {
        state.lastJobKey = jobKey;
        if (["completed", "stopped", "unconfirmed"].includes(snapshot.job.status)) {
          const results = stopResults(snapshot.job);
          const confirmed = results.length > 0 && results.every((result) => result.status === "confirmed") && snapshot.job.status !== "unconfirmed";
          message(confirmed ? `${jobLabel(snapshot.job)}。依据停止后的连续新鲜里程计近零反馈，并非固件 ACK 或物理停车回执。` : "里程计停车反馈未确认，请查看逐车原因；仍可点击“再次停止本次车辆”。", !confirmed);
        } else if (snapshot.job.status === "stopping") message("正在发送零速度并等待逐车里程计反馈；当前尚未满足全部车辆的反馈确认准则。");
        else if (snapshot.job.status === "failed") message(`任务失败：${snapshot.job.error || "请查看事件记录"}；停车结果请查看逐车反馈。`, true);
      }
      if (wasOffline && state.localEvents.some((entry) => entry.message.includes("服务连接中断"))) log("控制服务连接已恢复；没有自动重发任何任务。");
      if (snapshot.trajectory?.id) hydrateTrajectory(snapshot.trajectory.id);
    } catch (error) {
      if (state.online) log("控制服务连接中断；任务状态待确认，未自动重发命令。", "warning");
      state.online = false;
      state.connectionError = error.name === "AbortError" ? "读取状态超时" : error.message;
    } finally {
      state.polling = false;
      render();
      state.timer = setTimeout(poll, 500);
    }
  }

  async function run() {
    if (state.controlMode === "trajectory") return runTrajectory();
    const draft = commandDraft();
    if (draft.error || !state.csrf) return;
    // Freeze exactly the user's reviewed draft before any asynchronous work.
    const payload = JSON.stringify(draft);
    state.runPending = true;
    state.unknownOutcome = false;
    message("");
    log(`提交任务：${Object.keys(draft.commands).join("、")}，持续 ${draft.duration} 秒，20 Hz。`);
    render();
    try {
      const response = await request("/api/run", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: payload });
      if (response.job) state.snapshot.job = response.job;
      message("任务已提交。执行状态与剩余时间将在下方更新。");
    } catch (error) {
      state.unknownOutcome = !error.httpStatus || error.httpStatus >= 500;
      state.unknownSince = Date.now();
      if (!error.httpStatus) { state.online = false; state.connectionError = error.message; }
      const explanation = state.unknownOutcome ? "请求可能已到达服务，请查看任务状态或使用停止；不会自动重试。" : "服务已拒绝本次任务，未自动重试。";
      message(`启动请求未确认：${error.message}。${explanation}`, true);
      log(`启动请求未确认：${error.message}。`, "error");
    } finally {
      state.runPending = false;
      render();
      poll();
    }
  }

  async function stop(scope) {
    if (!state.csrf || state.stopPending) return;
    state.stopPending = true;
    renderControls();
    const label = scope === "all" ? "全部配置车辆" : "本次任务车辆";
    try {
      const response = await request("/api/stop", { method: "POST", headers: { "Content-Type": "application/json", "X-CSRF-Token": state.csrf }, body: JSON.stringify({ scope }) });
      if (response.job && state.snapshot) state.snapshot.job = response.job;
      state.unknownOutcome = false;
      message(`已接受${label}的停止请求，正在等待逐车里程计反馈；提交成功不代表停车已确认。`);
      log(`已向${label}请求发送零速度。`, "warning");
    } catch (error) {
      if (!error.httpStatus) { state.online = false; state.connectionError = error.message; }
      message(`停止请求未确认：${error.message}。无法确认零速度已送达，可再次点击停止。`, true);
      log(`停止请求未确认：${error.message}。`, "error");
    } finally {
      state.stopPending = false;
      render();
      poll();
    }
  }

  $("car-search").addEventListener("input", renderCars);
  ["velocity", "trajectory"].forEach((mode) => $("control-" + mode).addEventListener("click", () => { if (!busy()) { state.controlMode = mode; render(); } }));
  $("trajectory-file").addEventListener("change", importTrajectoryFile);
  $("trajectory-check").addEventListener("click", checkTrajectory);
  $("trajectory-reload").addEventListener("click", () => hydrateTrajectory(null, true));
  $("trajectory-select-all").addEventListener("click", () => { if (!busy() && !state.trajectoryBusy) { state.trajectorySelected = new Set(trajectoryCars().map((car) => car.car_id)); clearTrajectoryCheck(); render(); } });
  $("trajectory-clear").addEventListener("click", () => { if (!busy() && !state.trajectoryBusy) { state.trajectorySelected.clear(); clearTrajectoryCheck(); render(); } });
  $("trajectory-generate").addEventListener("click", generateTrajectory);
  $("trajectory-template").addEventListener("click", () => downloadText("car_id,t,x,y\n04,0,0,0\n04,5,0.15,0\n06,0,0,0.5\n06,5,0.15,0.5\n", "leap-world-template-adjust-before-use.csv"));
  $("online-only").addEventListener("change", renderCars);
  $("select-online").addEventListener("click", () => {
    if (busy() || !state.online) return;
    cars().filter(isConnected).forEach((car) => {
      state.selected.add(car.id);
      if (!state.individual.has(car.id)) state.individual.set(car.id, { v: $("uniform-v").value, w: $("uniform-w").value });
    });
    render();
  });
  $("clear-selection").addEventListener("click", () => { if (!busy()) { state.selected.clear(); render(); } });
  ["uniform", "individual"].forEach((mode) => $("mode-" + mode).addEventListener("click", () => { if (!busy()) { state.mode = mode; renderControls(); } }));
  ["uniform-v", "uniform-w", "duration"].forEach((id) => $(id).addEventListener("input", renderControls));
  document.querySelectorAll("[data-preset]").forEach((button) => button.addEventListener("click", () => {
    if (busy()) return;
    const values = { forward: [0.05, 0], backward: [-0.05, 0], left: [0, 0.15], right: [0, -0.15] }[button.dataset.preset];
    $("uniform-v").value = values[0]; $("uniform-w").value = values[1]; renderControls();
  }));
  $("run-button").addEventListener("click", run);
  $("stop-job").addEventListener("click", () => stop("job"));
  $("stop-all").addEventListener("click", () => stop("all"));
  $("show-job-results").addEventListener("click", () => {
    const results = $("job-panel");
    if (typeof results.scrollIntoView === "function") results.scrollIntoView({ behavior: "smooth", block: "nearest" });
  });
  $("refresh-button").addEventListener("click", poll);
  $("connection-select-all").addEventListener("click", () => {
    state.connectionSelected = new Set([...cars().map((car) => car.id), ...connectionRows().map((row) => row.id)]);
    renderConnections();
  });
  $("connection-clear").addEventListener("click", () => { state.connectionSelected.clear(); renderConnections(); });
  $("connection-check").addEventListener("click", () => connectionAction("check"));
  $("connection-connect").addEventListener("click", () => connectionAction("connect"));
  $("mocap-frame").addEventListener("change", () => { state.mocapFrame = $("mocap-frame").value; renderMocapMap(); });
  $("mocap-fit").addEventListener("click", () => { state.mocapBounds.delete(state.mocapFrame || ""); renderMocapMap(); });
  $("download-log").addEventListener("click", () => {
    const record = { exported_at: new Date().toISOString(), mode: state.snapshot?.mode || "unknown", service_online: state.online, last_state_received_at: state.receivedAt ? new Date(state.receivedAt).toISOString() : null, job: job(), cars: cars(), connections: connections(), events: [...state.localEvents, ...(state.snapshot?.events || [])] };
    const url = URL.createObjectURL(new Blob([JSON.stringify(record, null, 2)], { type: "application/json" }));
    const anchor = document.createElement("a");
    anchor.href = url;
    anchor.download = `leap-fleet-${new Date().toISOString().replace(/[:.]/g, "-")}.json`;
    anchor.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  setInterval(() => {
    if (state.online && Date.now() - state.receivedAt > 2000) {
      state.online = false;
      state.connectionError = "超过 2 秒未收到新状态";
      log("控制服务状态已过期；已禁用新任务，当前任务状态待确认。", "warning");
      render();
    }
  }, 250);
  render();
  poll();
})();
