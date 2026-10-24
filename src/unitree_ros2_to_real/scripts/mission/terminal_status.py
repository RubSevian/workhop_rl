"""Small Russian operator messages; original reason codes remain visible."""
import re

STATES={
    'IDLE':'Ожидание включения управления', 'WAIT_GOAL':'Ожидание новой цели навигации',
    'NAVIGATING':'Навигация к цели', 'BASE_SETTLE':'База остановлена; ожидание стабилизации',
    'ARM_SCAN_POSE':'Подъём руки в позу поиска', 'ARM_SEARCH':'Поиск объекта',
    'PLAN_GRASP':'Расчёт захвата; движение к объекту ещё не началось',
    'VIEW_REFINEMENT':'Уточнение положения камеры', 'PREGRASP':'Движение к позиции перед захватом',
    'GRASP':'Выполнение захвата', 'WAIT_RETREAT_APPROVAL':'Ожидание разрешения оператора на отход',
    'RETREAT':'Отход после захвата', 'RETURN_HOME':'Возврат руки HOME',
    'RETURN_SEARCH':'Возврат к позиции поиска', 'OBJECT_NOT_FOUND':'Объект не найден',
    'TARGET_NOT_GRASPED':'Объект не захвачен', 'GRASP_UNREACHABLE':'Подходящий захват не найден',
    'GRASP_PRESUMED_SUCCESS':'HOME завершён; захват предполагается по оценке контроллера',
    'MISSION_COMPLETED_UNVERIFIED':'Миссия завершена без подтверждения захвата'}
STAGES={'detection':'Получение кадров и обнаружение объекта',
    'graspnet':'GraspNet: построение вариантов захвата',
    'selection_including_collision':'Отбор вариантов: IK и проверка коллизий',
    'collision_pregrasp':'Проверка коллизий: путь к объекту',
    'collision_approach':'Проверка коллизий: подход губок',
    'collision_retreat':'Проверка коллизий: отход',
    'collision_close':'Проверка коллизий: закрытие губок',
    'collision_home':'Проверка коллизий: возврат HOME'}
OPERATIONS={'scan_pose':'поза поиска','detect':'обнаружение объекта','grasp_plan':'план захвата',
    'observe_grasp':'наблюдение захвата','pregrasp':'позиция перед захватом',
    'approach':'подход','close':'закрытие губок','retreat':'отход','home':'возврат HOME',
    'scan_step':'поворот поиска','scan_return':'возврат после поиска'}


def reason_text(reason):
    reason=str(reason or '')
    descriptions={'PHASE_TIMEOUT':'Истёк общий срок этапа',
        'CAMERA_FRAME_STALE':'Кадры камеры старше разрешённого возраста',
        'RGBD_STALE_OR_REPEATED':'Не получены новые пригодные RGB-D кадры',
        'RGBD_TF_FEEDBACK_STALE':'Не удалось согласовать кадр с измерениями робота',
        'IK_FAIL':'IK не нашёл допустимую позу для предложенных захватов',
        'NO_CANDIDATES':'После фильтров не осталось кандидатов захвата',
        'R3_AUTHORITY_REVOKED':'Контроллер базы отозвал разрешение на миссию',
        'COLLISION_CANCELLED:MISSION_CANCELLED':'Проверка отменена координатором миссии',
        'GRASPNET_WORKER_EXITED':'Процесс GraspNet завершился',
        'READY':'Данные готовы'}
    if reason.startswith('GraspNet inference exceeded '):return 'Превышено время ожидания GraspNet: '+reason
    if reason in descriptions:return descriptions[reason]
    match=re.search(r'COLLISION:([^:\s\'",}]+):([^:\s\'",}]+)',reason)
    if match:return 'Обнаружена коллизия: '+match[1]+' ↔ '+match[2]
    if reason.startswith('SCENE_SNAPSHOT_STALE:'):return 'Устарели измерения сцены: '+reason.split(':',1)[1]
    if reason.startswith('OBSERVATION_RECOVERY_TIMEOUT:'):return 'Истёк срок восстановления наблюдения'
    return reason


def seconds(value):
    return f'{value:.1f} с' if isinstance(value,(int,float)) else 'время неизвестно'


def terminal_lines(row):
    event=row.get('event')
    if event=='start':return []
    if event=='mission_state':
        state=row.get('state');reason=row.get('reason','');pending=row.get('pending') or {}
        if state in ('BLOCKED','CANCELLED'):
            label='ОСТАНОВКА' if state=='BLOCKED' else 'ОТМЕНА'
            duration=row.get('previous_phase_duration_s')
            suffix=('; этап длился '+seconds(duration)) if duration is not None else ''
            return [f'[{label}] {reason_text(reason)}{suffix}. Код: {reason}.']
        text=STATES.get(state,state or 'Состояние неизвестно')
        if pending.get('operation'):
            kind='расчёт' if pending.get('kind')=='compute' else 'движение'
            text+='; '+kind+': '+OPERATIONS.get(pending['operation'],pending['operation'])
        lines=['[МИССИЯ] '+text+'.']
        if reason=='TARGET_NOT_GRASPED_CONTINUING':
            lines.append('[ЗАХВАТ] Объект не схвачен; продолжаем миссию: отход и возврат HOME.')
        return lines
    if event=='planning_stage':
        name=row.get('stage');label=STAGES.get(name,name)
        if row.get('status')=='running':return ['[ПЛАН] '+label+'…']
        if row.get('status')=='completed':return ['[ПЛАН] '+label+': завершено за '+seconds(row.get('elapsed_s'))+'.']
        reason=row.get('reason','')
        return ['[ПЛАН] '+label+': '+reason_text(reason)+'; '+seconds(row.get('elapsed_s'))+'.']
    if event=='compute_result':
        summary=row.get('summary',{});operation=OPERATIONS.get(row.get('operation'),row.get('operation','расчёт'))
        outcome='готово' if row.get('ok') else reason_text(row.get('reason',''))
        if row.get('operation')=='detect' and row.get('ok'):
            outcome='объект найден' if row.get('found') else 'объект не найден'
        lines=[f'[РАСЧЁТ] {operation}: {outcome}; {seconds(summary.get("elapsed_s"))}.']
        counts=summary.get('filter_counts',{})
        if 'decoded' in counts:
            lines.append('[ОТБОР] GraspNet: '+str(counts.get('decoded'))+
                '; после фильтров: '+str(counts.get('after_depth'))+
                '; после NMS: '+str(counts.get('after_nms','—'))+
                '; вариантов после IK: '+str(counts.get('ik_feasible','—'))+'.')
        return lines
    return []
