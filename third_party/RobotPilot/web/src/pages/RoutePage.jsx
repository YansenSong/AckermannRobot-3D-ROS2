import React, { useRef, useState, useEffect, useCallback } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos } from "../app/App";
import { useRoleAccess } from "../shared/auth/roleAccess";
import { AppConfig } from "../shared/constants/index";
import { useT } from "../shared/i18n/i18n";
import TimeModal from "../components/modal/TimeModal";
import RouteModal from "../components/modal/RouteModal";
import TextInputModal from "../components/modal/TextInputModal";

import Map from "../components/Map";
import MapLayers from "../components/MapLayers";
import Button from "../shared/ui/Button";
import {
  DashboardCard,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";

const removeCsv = (data) => {
  if (Array.isArray(data)) {
    return data.map((item) => {
      if (typeof item === "string") {
        return item.replace(".csv", "");
      }
      return removeCsv(item);
    });
  } else if (typeof data === "object" && data !== null) {
    const result = {};
    for (const key in data) {
      result[key] = removeCsv(data[key]);
    }
    return result;
  }
  return data;
};

function replaceUnderscoresInKeysAndValues(obj) {
  if (Array.isArray(obj)) {
    return obj.map((item) => replaceUnderscoresInKeysAndValues(item));
  } else if (typeof obj === "object" && obj !== null) {
    const newObj = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        const newKey = key.replace(/_/g, " ");
        newObj[newKey] = replaceUnderscoresInKeysAndValues(obj[key]);
      }
    }
    return newObj;
  } else if (typeof obj === "string") {
    return obj.replace(/_/g, " ");
  }
  return obj;
}

const processObjectStrings = (obj) => {
  if (typeof obj === "object" && obj !== null) {
    const result = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        result[key] = processObjectStrings(obj[key]);
      }
    }
    return result;
  } else if (typeof obj === "string") {
    return obj.trim().replace(/\s/g, "_");
  }
  return obj;
};

const findMapArray = (structure, activeFiles) => {
  const groupObject = structure.find((floor) =>
    Object.prototype.hasOwnProperty.call(floor, activeFiles.group),
  );

  if (groupObject) {
    const mapsArray = groupObject[activeFiles.group];

    const mapObject = mapsArray.find((room) =>
      Object.prototype.hasOwnProperty.call(room, activeFiles.map),
    );

    if (mapObject) {
      return mapObject[activeFiles.map];
    }
  }

  return [];
};

const RoutePage = () => {
  const { t } = useT();
  const ros = useRos();
  const routeAccess = useRoleAccess("Engineer");
  // eslint-disable-next-line no-unused-vars
  const [selectedPointType, setSelectedPointType] = useState(null);
  const [pointsSettable, setPointsSettable] = useState(false);
  const [planningRoute, setPlanningRoute] = useState(false);
  // eslint-disable-next-line no-unused-vars
  const [hoursValue, setHoursValue] = useState("0");
  const latestHoursValue = useRef(hoursValue);
  // eslint-disable-next-line no-unused-vars
  const [minutesValue, setMinutesValue] = useState("0");
  const latestMinutesValue = useRef(minutesValue);

  useEffect(() => {
    latestHoursValue.current = hoursValue;
    latestMinutesValue.current = minutesValue;
  }, [hoursValue, minutesValue]);

  const [selectedFile, setSelectedFile] = useState({
    group: "",
    map: "",
    route: "",
  });
  const [openRouteModal, setOpenRouteModal] = useState(false);
  const [openTimeModal, setOpenTimeModal] = useState(false);
  const [openInputModal, setOpenInputModal] = useState(false);

  const [filesData, setFilesData] = useState([]);
  const [routeWaypoints, setRouteWaypoints] = useState([]);

  useEffect(() => {
    window.NAV2D?.setQueuedWaypoints?.(
      routeWaypoints.map((point) => ({
        position: { x: point.x, y: point.y },
        orientation: {
          x: point.qx,
          y: point.qy,
          z: point.qz,
          w: point.qw,
        },
      })),
    );
    return () => window.NAV2D?.clearQueuedWaypoints?.();
  }, [routeWaypoints]);

  const childRef = useRef(null);
  const routesModalType = useRef(null);
  const isRoutesModalWithInput = useRef(null);
  const routesModalHeader = useRef(null);
  const modalKey = useRef(null);

  const textInputHeader = useRef(null);
  const textInputPlaceholder = useRef(null);
  const hasActiveMap =
    Boolean(selectedFile.group) &&
    selectedFile.group !== "Null" &&
    Boolean(selectedFile.map) &&
    selectedFile.map !== "Null";

  /* TOPICS */

  const filesReqTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROUTE_DATA_REQ_TOPIC,
      messageType: "std_msgs/Empty",
    }),
  );

  const filesResonseTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROUTE_DATA_RESP_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const uiOperationTopic = useRef(
    new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    }),
  );

  const onMapClickHandler = useCallback(() => {
    if (!window.NAV2D.arePointsSettable) return;

    // Open the time configuration modal upon placing a waypoint
    setOpenTimeModal(true);
  }, []);

  useEffect(() => {
    window.NAV2D.ClearMap();

    const currentFilesResponseTopic = filesResonseTopic.current;
    const currentMap = childRef.current;

    currentFilesResponseTopic.subscribe((data) => {
      const response = data.data;
      const responseObject = JSON.parse(response);

      const serializedArray = removeCsv(responseObject.structure);
      const arrayWithSpaces =
        replaceUnderscoresInKeysAndValues(serializedArray);

      // const serializedActiveFile = {
      //   group: responseObject.active_files.group,
      //   map: responseObject.active_files.map,
      //   route: responseObject.active_files.route.replace(".csv", ""),
      // };

      const activeFilesWithSpaces = replaceUnderscoresInKeysAndValues({
        group: responseObject.active_files.group,
        map: responseObject.active_files.map,
        route: responseObject.active_files.route.replace(".csv", ""),
      });

      const filtredArrayBySelectedRoute = findMapArray(
        arrayWithSpaces,
        activeFilesWithSpaces,
      );

      setFilesData(filtredArrayBySelectedRoute);
      setSelectedFile((current) => {
        const sameMap =
          current.group === activeFilesWithSpaces.group &&
          current.map === activeFilesWithSpaces.map;
        const selectedRoute =
          sameMap &&
          (current.route === "New route" ||
            filtredArrayBySelectedRoute.includes(current.route))
            ? current.route
            : "Null";
        return { ...activeFilesWithSpaces, route: selectedRoute };
      });
    });

    filesReqTopic.current.publish();

    const currentWaypointsTopic = new window.ROSLIB.Topic({
      ros,
      name: "/WayPoints_topic",
      messageType: "robotpilot_ui_msgs/ArrayPoseStampedWithCovariance",
    });
    currentWaypointsTopic.subscribe((message) => {
      const points = (message?.poses || []).map((entry) => {
        const pose = entry?.pose?.pose;
        const covariance = entry?.pose?.covariance || [];
        return {
          x: Number(pose?.position?.x || 0),
          y: Number(pose?.position?.y || 0),
          qx: Number(pose?.orientation?.x || 0),
          qy: Number(pose?.orientation?.y || 0),
          qz: Number(pose?.orientation?.z || 0),
          qw: Number(pose?.orientation?.w ?? 1),
          hours: Number(covariance[1] || 0),
          minutes: Number(covariance[2] || 0),
        };
      });
      setRouteWaypoints(points);
    });

    return () => {
      currentFilesResponseTopic.unsubscribe();
      currentWaypointsTopic.unsubscribe();
      window.NAV2D.arePointsSettable = false;

      const mapElement = currentMap ? currentMap.getMapRef() : null;
      if (mapElement) {
        mapElement.removeEventListener("mouseup", onMapClickHandler);
      }
    };
  }, [onMapClickHandler, ros]);

  const onOperationTopicPublish = (message) => {
    if (!routeAccess.allowed) {
      toast.warn(t(routeAccess.reason));
      return false;
    }
    if (!uiOperationTopic.current) return false;
    uiOperationTopic.current.publish(
      new window.ROSLIB.Message({ data: message }),
    );
    return true;
  };

  const selectRouteForEditing = (route) => {
    if (!route || !hasActiveMap) return;
    window.NAV2D.arePointsSettable = false;
    setPointsSettable(false);
    window.NAV2D.clearQueuedWaypoints?.();
    window.NAV2D.ClearMap();
    const nextFile = { ...selectedFile, route };
    setSelectedFile(nextFile);
    onOperationTopicPublish(
      `change_route/${JSON.stringify(processObjectStrings(nextFile))}`,
    );
  };

  const publishEditedWaypoints = (points) => {
    if (!routeAccess.allowed) {
      toast.warn(t(routeAccess.reason));
      return;
    }
    const topic = uiOperationTopic.current;
    if (!topic) return;
    const waypoints = points.map(({ x, y, qx, qy, qz, qw, hours, minutes }) => [
      Number(x),
      Number(y),
      0,
      Number(qx) || 0,
      Number(qy) || 0,
      Number(qz) || 0,
      Number(qw ?? 1),
      3,
      Number(hours) || 0,
      Number(minutes) || 0,
    ]);
    topic.publish(
      new window.ROSLIB.Message({
        data: `replace_route/${JSON.stringify({ waypoints })}`,
      }),
    );
  };

  const updateRouteWaypoint = (index, field, value) => {
    setRouteWaypoints((current) =>
      current.map((point, pointIndex) =>
        pointIndex === index ? { ...point, [field]: value } : point,
      ),
    );
  };

  const removeRouteWaypoint = (index) => {
    const next = routeWaypoints.filter((_, pointIndex) => pointIndex !== index);
    setRouteWaypoints(next);
    publishEditedWaypoints(next);
  };

  /* FROM HANDLERS */

  const onRouteFormSubmitHandler = (data) => {
    setOpenRouteModal(false);

    if (data) {
      const operationsConfig = {
        CHANGE_ROUTE: {
          path: "change_route",
          data: {
            group: selectedFile.group,
            map: selectedFile.map,
            route: data,
          },
          preActions: () => window.NAV2D.ClearMap(),
          postActions: () =>
            setSelectedFile({
              group: selectedFile.group,
              map: selectedFile.map,
              route: data,
            }),
        },
      };

      const currentOperation = operationsConfig[modalKey.current];
      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();
        const objToSendWithoutSpaces = processObjectStrings(
          currentOperation.data,
        );

        const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);
        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        onOperationTopicPublish(messageToSend);
        currentOperation.postActions && currentOperation.postActions();
      }
    }

    modalKey.current = null;
    routesModalType.current = null;
    isRoutesModalWithInput.current = null;
    routesModalHeader.current = null;
  };

  // TODELETE
  const onTimeFormSubmitHandler = (data) => {
    setOpenTimeModal(false);

    if (data) {
      if (!routeAccess.allowed) {
        toast.warn(t(routeAccess.reason));
        return;
      }
      window.NAV2D.sendPointToRobot(ros, data);
    } else {
      const markerOnMap = window.NAV2D.orientatedPointItem;
      window.NAV2D.pointsArray = window.NAV2D.pointsArray.filter(
        (marker) => marker !== markerOnMap,
      );
      window.NAV2D.canvas.scene.removeChild(markerOnMap);
      window.NAV2D.finishedPointItem = null;
      window.NAV2D.orientatedPointItem = null;
    }
  };

  const onInputFormSubmitHandler = (data) => {
    setOpenInputModal(false);

    if (data) {
      const operationsConfig = {
        SAVE_ROUTE: {
          path: "save_route",
          data: {
            group: selectedFile.group,
            map: selectedFile.map,
            route: data,
          },

          postActions: () => {
            const mapElement = childRef.current.getMapRef();

            if (mapElement) {
              mapElement.removeEventListener("mouseup", onMapClickHandler);
            }

            setPointsSettable(false);
            setSelectedFile((prev) => ({ ...prev, route: data }));
            setSelectedPointType(null);
            window.NAV2D.pointType = null;
            window.NAV2D.arePointsSettable = false;
          },
        },

        RENAME_ROUTE: {
          path: "rename_route",
          data: {
            group: selectedFile.group,
            map: selectedFile.map,
            route_old: selectedFile.route,
            route_new: data,
          },
          postActions: () => {
            const newFileData = {
              group: selectedFile.group,
              map: selectedFile.map,
              route: data,
            };
            setSelectedFile(newFileData);
          },
        },
      };

      const currentOperation = operationsConfig[modalKey.current];

      if (currentOperation) {
        currentOperation.preActions && currentOperation.preActions();

        const objToSendWithoutSpaces = processObjectStrings(
          currentOperation.data,
        );
        const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

        const messageToSend = `${currentOperation.path}/${stringifiedObjToSend}`;
        onOperationTopicPublish(messageToSend);
        currentOperation.postActions && currentOperation.postActions();
      }
    }

    modalKey.current = null;
    textInputHeader.current = null;
    textInputPlaceholder.current = null;
  };

  /* BUTTON HANDLERS */

  const onNewRouteClick = () => {
    if (!hasActiveMap) {
      toast.warn(t("Waiting for the active simulation map."));
      return;
    }

    setSelectedPointType(null);
    setSelectedFile({
      group: selectedFile.group,
      map: selectedFile.map,
      route: "New route",
    });

    window.NAV2D.pointType = null;
    window.NAV2D.arePointsSettable = true;

    const mapElement = childRef.current.getMapRef();

    if (mapElement) {
      mapElement.addEventListener("mouseup", onMapClickHandler);
    }

    setPointsSettable(true);
    window.NAV2D.ClearMap();
    onOperationTopicPublish("clear_route");
  };

  const onPlanRouteClick = async () => {
    if (!routeAccess.allowed) {
      toast.warn(t(routeAccess.reason));
      return;
    }
    if (!ros) {
      toast.error(t("Robot connection is offline!"));
      return;
    }

    const currentPose = window.NAV2D.currentPose;
    if (!currentPose) {
      toast.error(
        t(
          "Waiting for the robot's current position — make sure it's localized on the map, then try again.",
        ),
      );
      return;
    }

    if (!routeWaypoints.length) {
      toast.warn(t("Add or load route waypoints first."));
      return;
    }
    if (planningRoute) return;

    const planSegment = (start, goal, segmentNumber) =>
      new Promise((resolve, reject) => {
        const id =
          window.crypto?.randomUUID?.() || `${Date.now()}-${segmentNumber}`;
        const responseTopic = new window.ROSLIB.Topic({
          ros,
          name: AppConfig.ROUTE_PLAN_RESPONSE_TOPIC,
          messageType: "std_msgs/String",
        });
        const requestTopic = new window.ROSLIB.Topic({
          ros,
          name: AppConfig.ROUTE_PLAN_REQUEST_TOPIC,
          messageType: "std_msgs/String",
        });
        const cleanup = () => {
          window.clearTimeout(timeout);
          responseTopic.unsubscribe();
        };
        const timeout = window.setTimeout(() => {
          cleanup();
          reject(
            new Error(
              t("Global planner request failed for segment {segment}.").replace(
                "{segment}",
                segmentNumber,
              ),
            ),
          );
        }, 45000);
        responseTopic.subscribe((message) => {
          let result;
          try {
            result = JSON.parse(message.data);
          } catch {
            return;
          }
          if (result.id !== id) return;
          cleanup();
          if (result.error) {
            reject(new Error(`${segmentNumber}: ${result.error}`));
          } else if (Array.isArray(result.poses) && result.poses.length) {
            resolve(result.poses);
          } else {
            reject(new Error(t("Couldn't find a route to that point.")));
          }
        });
        requestTopic.publish(
          new window.ROSLIB.Message({
            data: JSON.stringify({ id, start, goal }),
          }),
        );
      });

    setPlanningRoute(true);
    window.NAV2D.clearPath?.();
    const plannedPath = [];
    let start = currentPose;
    try {
      for (let index = 0; index < routeWaypoints.length; index += 1) {
        toast.info(
          t("Planning segment {current} / {total}…")
            .replace("{current}", index + 1)
            .replace("{total}", routeWaypoints.length),
        );
        const point = routeWaypoints[index];
        const goal = {
          position: { x: Number(point.x), y: Number(point.y), z: 0 },
          orientation: {
            x: Number(point.qx) || 0,
            y: Number(point.qy) || 0,
            z: Number(point.qz) || 0,
            w: Number(point.qw ?? 1),
          },
        };
        const segment = await planSegment(start, goal, index + 1);
        plannedPath.push(...(index ? segment.slice(1) : segment));
        start = goal;
      }

      window.NAV2D.drawRoutePlan?.(plannedPath);
      toast.success(
        t("Planned {count} route segments with the global planner.").replace(
          "{count}",
          routeWaypoints.length,
        ),
      );
    } catch (error) {
      window.NAV2D.clearPath?.();
      console.error("Route planning failed:", error);
      toast.error(error?.message || t("Global route planning failed."));
    } finally {
      setPlanningRoute(false);
    }
  };

  const onSaveRouteClick = () => {
    if (!pointsSettable) return;

    if (selectedFile.route !== "New route") {
      publishEditedWaypoints(routeWaypoints);
      const dataToSend = {
        group: selectedFile.group,
        map: selectedFile.map,
        route: selectedFile.route,
      };

      const objToSendWithoutSpaces = processObjectStrings(dataToSend);
      const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

      const messageToSend = `save_route/${stringifiedObjToSend}`;
      onOperationTopicPublish(messageToSend);

      setSelectedPointType(null);
      window.NAV2D.pointType = null;
      window.NAV2D.arePointsSettable = false;

      const mapElement = childRef.current.getMapRef();

      if (mapElement) {
        mapElement.removeEventListener("mouseup", onMapClickHandler);
      }

      setPointsSettable(false);
      return;
    }

    modalKey.current = "SAVE_ROUTE";
    textInputHeader.current = t("Enter new route name");
    textInputPlaceholder.current = t("Name...");
    setOpenInputModal(true);
  };

  const onChangeRouteClick = () => {
    console.log("filesData", filesData);
    window.NAV2D.arePointsSettable = false;
    setPointsSettable(false);

    modalKey.current = "CHANGE_ROUTE";
    routesModalType.current = "selectRoute";
    isRoutesModalWithInput.current = false;
    routesModalHeader.current = t("Select route you want to browse");
    setOpenRouteModal(true);
  };

  const onEditRouteClick = () => {
    if (pointsSettable) {
      window.NAV2D.arePointsSettable = false;
      const mapElement = childRef.current?.getMapRef();
      mapElement?.removeEventListener("mouseup", onMapClickHandler);
      setPointsSettable(false);
    } else {
      window.NAV2D.ClearMap();

      const currentRouteData = {
        group: selectedFile.group,
        map: selectedFile.map,
        route: selectedFile.route,
      };

      const objToSendWithoutSpaces = processObjectStrings(currentRouteData);
      const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

      const messageToSend = `edit_route/${stringifiedObjToSend}`;

      onOperationTopicPublish(messageToSend);

      window.NAV2D.arePointsSettable = true;

      const mapElement = childRef.current.getMapRef();
      if (mapElement) {
        mapElement.addEventListener("mouseup", onMapClickHandler);
      }

      setPointsSettable(true);
    }
  };

  const onClearRouteClick = () => {
    if (!pointsSettable) return;
    window.NAV2D.ClearMap();
    onOperationTopicPublish("clear_route");
  };

  const onDeleteRouteClick = () => {
    window.NAV2D.ClearMap();

    const currentRouteData = {
      group: selectedFile.group,
      map: selectedFile.map,
      route: selectedFile.route,
    };

    const objToSendWithoutSpaces = processObjectStrings(currentRouteData);
    const stringifiedObjToSend = JSON.stringify(objToSendWithoutSpaces);

    const messageToSend = `delete_route/${stringifiedObjToSend}`;

    onOperationTopicPublish(messageToSend);

    // modalKey.current = "DELETE_ROUTE";
    // routesModalType.current = "selectRoute";
    // isRoutesModalWithInput.current = false;
    // routesModalHeader.current = "Select route you want to delete";
    // setOpenRouteModal(true);
  };

  const onRenameRouteClick = () => {
    // console.log(!pointsSettable || selectedFile.route === "New route");
    // if (selectedFile.route === "New route") return;
    modalKey.current = "RENAME_ROUTE";
    textInputHeader.current = t("Enter route new name");
    textInputPlaceholder.current = t("Name...");
    setOpenInputModal(true);
  };

  // eslint-disable-next-line no-unused-vars
  const onPointClickHandler = (data) => {
    setSelectedPointType(data);
    window.NAV2D.pointType = data;
  };

  return (
    <>
      <ToastContainer position="bottom-right" theme="dark" />

      {openRouteModal && (
        <RouteModal
          routesList={
            modalKey.current === "CHANGE_MAP" ? mapOptions : filesData
          }
          headerText={routesModalHeader.current}
          modalHandler={onRouteFormSubmitHandler}
        />
      )}

      {openTimeModal && <TimeModal modalHandler={onTimeFormSubmitHandler} />}

      {openInputModal && (
        <TextInputModal
          header={textInputHeader.current}
          placeholder={textInputPlaceholder.current}
          routesList={filesData}
          modalHandler={onInputFormSubmitHandler}
        />
      )}

      <div className="sectionHeight flex flex-col gap-5 py-4 sm:gap-6 sm:py-6 xl:h-[calc(100vh-145px)] xl:min-h-0">
        <SectionHeader
          eyebrow="Route authoring"
          title="Task planning"
          description="Create and manage reusable routes for the map currently loaded in the Ackermann simulation."
          action={
            <StatusBadge
              status={pointsSettable ? "active" : "idle"}
              label={pointsSettable ? "Editing route" : "View mode"}
              pulse={pointsSettable}
            />
          }
        />
        {!routeAccess.allowed && (
          <p className="rounded-lg border border-statusYellow/40 p-3 text-xs text-statusYellow">
            {t(routeAccess.reason)}.{" "}
            {t("Route authoring requires Engineer permission.")}
          </p>
        )}

        <div className="grid grid-cols-1 gap-3 sm:grid-cols-2">
          {[
            ["Map", selectedFile.map],
            ["Current route", selectedFile.route],
          ].map(([label, value, caption]) => (
            <DashboardCard key={label} className="min-w-0 px-4 py-3">
              <p
                className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.14em] text-themeTextGray"
                title={caption}
              >
                {t(label)}
              </p>
              <p
                className="mt-1 truncate font-[RobotoMono] text-sm font-semibold text-textWhiteHover"
                title={value || "Not available"}
              >
                {value || "—"}
              </p>
            </DashboardCard>
          ))}
        </div>

        <MapLayers />

        <div className="grid min-w-0 flex-1 gap-4 xl:min-h-0 xl:grid-cols-[minmax(0,1fr)_380px] xl:gap-5">
          <section className="h-[440px] min-w-0 sm:h-[560px] xl:h-full xl:min-h-[500px]">
            <Map ref={childRef} />
          </section>

          <DashboardCard className="h-fit min-w-0 p-4 sm:p-5">
            <div className="mb-4 border-b border-borderSubtle pb-4">
              <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
                {t("Route operations")}
              </p>
              <p className="mt-2 text-sm leading-6 text-themeTextGray">
                {t(
                  pointsSettable
                    ? "Click the map to add or adjust waypoints, then save your changes."
                    : "Choose an operation to begin editing the current route or create a new one.",
                )}
              </p>
            </div>

            <div className="space-y-4">
              <div className="space-y-2 rounded-xl border border-borderSubtle bg-bgSurface/50 p-3">
                <p className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.12em] text-themeTextGray">
                  {t("Route selection")}
                </p>
                <Button
                  onBtnClick={onNewRouteClick}
                  type={
                    !routeAccess.allowed || pointsSettable || !hasActiveMap
                      ? "disabled"
                      : "orange"
                  }
                >
                  <span className="iconPlus" aria-hidden="true" />
                  <span>{t("Create")}</span>
                </Button>
                <label className="flex flex-col gap-1.5 font-[RobotoMono] text-xs text-themeTextGray">
                  {t("Choose a saved route to edit")}
                  <select
                    value={
                      selectedFile.route === "Null" ||
                      selectedFile.route === "New route"
                        ? ""
                        : selectedFile.route
                    }
                    onChange={(event) =>
                      selectRouteForEditing(event.target.value)
                    }
                    disabled={
                      !routeAccess.allowed ||
                      pointsSettable ||
                      filesData.length === 0 ||
                      !hasActiveMap
                    }
                    className="min-h-10 w-full rounded-lg border border-borderSubtle bg-bgCard px-3 text-sm text-textWhiteHover outline-none focus:border-themeBlue disabled:opacity-50"
                  >
                    <option value="" disabled hidden>
                      {t("Select a saved route…")}
                    </option>
                    {filesData.map((route) => (
                      <option key={route} value={route}>
                        {route}
                      </option>
                    ))}
                  </select>
                </label>
              </div>

              <div className="space-y-2 rounded-xl border border-borderSubtle bg-bgSurface/50 p-3">
                <p className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.12em] text-themeTextGray">
                  {t("Edit and save")}
                </p>
                <div className="grid grid-cols-2 gap-2">
                  <Button
                    onBtnClick={onEditRouteClick}
                    type={
                      !routeAccess.allowed ||
                      (!pointsSettable && selectedFile.route === "Null")
                        ? "disabled"
                        : ""
                    }
                  >
                    <span className="iconMap" aria-hidden="true" />
                    <span>
                      {t(pointsSettable ? "Cancel edit" : "Edit route")}
                    </span>
                  </Button>
                  <Button
                    onBtnClick={onSaveRouteClick}
                    type={
                      routeAccess.allowed && pointsSettable
                        ? "success"
                        : "disabled"
                    }
                  >
                    <span className="iconSave" aria-hidden="true" />
                    <span>{t("Save")}</span>
                  </Button>
                </div>
                <Button
                  onBtnClick={onClearRouteClick}
                  type={
                    !routeAccess.allowed || !pointsSettable ? "disabled" : ""
                  }
                >
                  <span className="iconTrash" aria-hidden="true" />
                  <span>{t("Clear waypoints")}</span>
                </Button>
              </div>

              <div className="space-y-2 rounded-xl border border-borderSubtle bg-bgSurface/50 p-3">
                <p className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.12em] text-themeTextGray">
                  {t("Path planning")}
                </p>
                <Button
                  onBtnClick={onPlanRouteClick}
                  type={
                    !routeAccess.allowed ||
                    planningRoute ||
                    !hasActiveMap ||
                    routeWaypoints.length === 0
                      ? "disabled"
                      : ""
                  }
                >
                  <span className="iconMap" aria-hidden="true" />
                  <span>
                    {t(planningRoute ? "Planning route…" : "Auto-plan")}
                  </span>
                </Button>
              </div>

              <div className="space-y-2 rounded-xl border border-borderSubtle bg-bgSurface/50 p-3">
                <p className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.12em] text-themeTextGray">
                  {t("Manage saved route")}
                </p>
                <div className="grid grid-cols-2 gap-2">
                  <Button
                    onBtnClick={onRenameRouteClick}
                    type={
                      !routeAccess.allowed ||
                      pointsSettable ||
                      selectedFile.route === "Null"
                        ? "disabled"
                        : ""
                    }
                  >
                    <span className="iconMap" aria-hidden="true" />
                    <span>{t("Rename")}</span>
                  </Button>
                  <Button
                    onBtnClick={onDeleteRouteClick}
                    type={
                      !routeAccess.allowed ||
                      pointsSettable ||
                      selectedFile.route === "Null"
                        ? "disabled"
                        : "danger"
                    }
                  >
                    <span className="iconTrash" aria-hidden="true" />
                    <span>{t("Delete")}</span>
                  </Button>
                </div>
              </div>
            </div>

            {pointsSettable && routeWaypoints.length > 0 && (
              <div className="mt-4 border-t border-borderSubtle pt-4">
                <div className="mb-3 flex items-center justify-between gap-2">
                  <p className="font-[RobotoMono] text-[10px] font-bold uppercase tracking-[0.14em] text-themeTextGray">
                    {t("Route waypoints")} ({routeWaypoints.length})
                  </p>
                  <Button
                    type="success"
                    onBtnClick={() => publishEditedWaypoints(routeWaypoints)}
                  >
                    {t("Apply edits")}
                  </Button>
                </div>
                <div className="max-h-64 space-y-2 overflow-y-auto pr-1">
                  {routeWaypoints.map((point, index) => (
                    <div
                      key={index}
                      className="grid grid-cols-2 gap-2 rounded-lg border border-borderSubtle p-2"
                    >
                      <p className="col-span-2 text-xs font-semibold text-textWhiteHover">
                        {t("Waypoint")} {index + 1}
                      </p>
                      {["x", "y", "hours", "minutes"].map((field) => (
                        <label
                          key={field}
                          className="flex flex-col gap-1 text-[10px] text-themeTextGray"
                        >
                          {t(
                            field === "x"
                              ? "X (m)"
                              : field === "y"
                              ? "Y (m)"
                              : field === "hours"
                              ? "Stop hours"
                              : "Stop minutes",
                          )}
                          <input
                            type="number"
                            min={0}
                            max={
                              field === "hours"
                                ? 23
                                : field === "minutes"
                                ? 59
                                : undefined
                            }
                            step={field === "x" || field === "y" ? "0.01" : "1"}
                            value={point[field]}
                            onChange={(event) =>
                              updateRouteWaypoint(
                                index,
                                field,
                                event.target.value,
                              )
                            }
                            className="min-h-9 min-w-0 rounded-md border border-borderSubtle bg-bgCard px-2 text-xs text-textWhiteHover outline-none focus:border-themeBlue"
                          />
                        </label>
                      ))}
                      <button
                        type="button"
                        onClick={() => removeRouteWaypoint(index)}
                        className="col-span-2 min-h-8 text-left text-xs text-statusRed hover:underline"
                      >
                        {t("Remove waypoint")}
                      </button>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </DashboardCard>
        </div>
      </div>
    </>
  );
};

export default RoutePage;

{
  /*
          <Button  onBtnClick={() => onPointClickHandler("home")}>
            <span className="iconPoint" />
            <span
              className={`mx-auto ${
                selectedPointType === "home" ? "text-green-600" : "color-white"
              }`}
            >
              Home point
            </span>
          </Button>
          <Button  onBtnClick={() => onPointClickHandler("charge")}>
            <span className="iconCharge" />
            <span
              className={`mx-auto ${
                selectedPointType === "charge"
                  ? "text-green-600"
                  : "color-white"
              }`}
            >
              Charge point
            </span>
          </Button>
          <Button  onBtnClick={() => onPointClickHandler("navigate")}>
            <span className="iconInfo" />
            <span
              className={`mx-auto ${
                selectedPointType === "navigate"
                  ? "text-green-600"
                  : "color-white"
              }`}
            >
              Nav point
            </span>
          </Button>
           */
}
