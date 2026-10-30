"""Small Russian operator messages; original reason codes remain visible."""
import re

STATES={
    'IDLE':'Ожидание включения управления', 'WAIT_GOAL':'Ожидание новой цели навигации',
    'NAVIGATING':'Навигация к цели', 'BASE_SETTLE':'База остановлена; ожидание стабилизации',
    'ARM_SCAN_POSE':'Подъём руки в позу поиска', 'ARM_SEARCH':'Поиск объекта',
    'PLAN_GRASP':'Расчёт захвата; движение к объекту ещё не началось',
    'VIEW_REFINEMENT':'Уточнение положения камеры', 'PREGRASP':'Движение к позиции перед захватом',
    'GRASP':'Выполнение захвата', 'WAIT_RETREAT_APPROVAL':'Ожидание разрешения оператора на отход',
    'RETREAT':'Отход и возврат в сохранённую позу сканирования', 'RETURN_HOME':'Возврат руки HOME',
    'RETURN_NAV_PREPARE':'Подготовка возврата базы после HOME',
    'RETURN_NAVIGATING':'Навигация к сохранённой стартовой позиции',
    'RETURN_NAV_SETTLE':'Подтверждение остановки в стартовой позиции',
    'RETURNED_TO_START':'Возврат базы завершён по одометрии',
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
    'collision_return_scan':'Проверка коллизий: возврат в сохранённую позу сканирования',
    'collision_home':'Проверка коллизий: возврат HOME'}
OPERATIONS={'retry_home_open':'открытие гриппера в HOME перед повторной попыткой','scan_pose':'поза поиска','detect':'обнаружение объекта','grasp_plan':'план захвата',
    'observe_grasp':'наблюдение захвата','pregrasp':'позиция перед захватом',
    'approach':'подход','close':'закрытие губок','retreat':'отход','home':'возврат HOME',
    'scan_step':'поворот поиска','scan_return':'возврат после поиска'}


def reason_text(reason):
    if reason=='NO_TARGET_BOUND_GRASPS':return 'У кандидатов не подтверждена поверхность нужного предмета между губками'
    if reason=='TARGET_BINDING_SEGMENTATION_REQUIRED':return 'Для привязки захвата нужна маска сегментации; рамка недостаточна'
    if reason=='CONTROLLED_STOP_PREPARING_GEOMETRY':return 'Остановка распознавания; подготовка расчёта возврата руки'
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


def block(title,*fields):
    return ['\n'+title+'\n'+'\n'.join('  '+field for field in fields)+'\n']


def scene_status_line(reason):
    fields=['Статус: '+reason_text(reason)]
    if reason.startswith('SCENE_SNAPSHOT_STALE:'):
        fields+=['Текущие измерения не готовы; это не итог расчёта.',
                 'Остановка миссии обозначается [ОСТАНОВКА] / [ОТМЕНА].']
    fields.append('Код: '+reason)
    return block('[ДАННЫЕ]',*fields)[0]


def terminal_lines(row):
    event=row.get('event')
    if event=='mission_state':
        state=row.get('state');reason=row.get('reason','');pending=row.get('pending') or {}
        if state in ('BLOCKED','CANCELLED'):
            label='ОСТАНОВКА' if state=='BLOCKED' else 'ОТМЕНА'
            fields=['Причина: '+reason_text(reason)]
            if row.get('previous_phase_duration_s') is not None:
                fields.append('Этап длился: '+seconds(row['previous_phase_duration_s']))
            return block('['+label+']',*fields,'Код: '+reason)
        fields=['Статус: '+STATES.get(state,state or 'Состояние неизвестно')]
        if row.get('full_attempt') is not None:
            fields.append('Попытка: '+str(row['full_attempt'])+' / 2')
        if reason=='SECOND_ATTEMPT_AFTER_CONFIRMED_HOME':
            fields.append('HOME подтверждён; начинаем новый поиск после первой неудачи.')
        if pending.get('operation'):
            kind='Расчёт' if pending.get('kind')=='compute' else 'Движение'
            fields.append(kind+': '+OPERATIONS.get(pending['operation'],pending['operation']))
        if reason=='TARGET_NOT_GRASPED_CONTINUING':
            fields+=['Захват: объект не схвачен.', 'Далее: отход и возврат HOME.']
        return block('[МИССИЯ]',*fields)
    if event=='planning_stage':
        name=row.get('stage');label=STAGES.get(name,name)
        status=row.get('status')
        fields=['Статус: '+{'running':'выполняется','completed':'завершено',
            'cancelled':'отменено','failed':'ошибка'}.get(status,str(status))]
        if row.get('elapsed_s') is not None:fields.append('Время: '+seconds(row['elapsed_s']))
        if row.get('reason'):
            fields+=['Причина: '+reason_text(row['reason']),'Код: '+row['reason']]
        return block('[ПЛАН] '+label,*fields)
    if event=='compute_result':
        summary=row.get('summary',{});operation=OPERATIONS.get(row.get('operation'),row.get('operation','расчёт'))
        outcome='готово' if row.get('ok') else reason_text(row.get('reason',''))
        if row.get('operation')=='detect' and row.get('ok'):
            outcome='объект найден' if row.get('found') else 'объект не найден'
        fields=['Результат: '+outcome,'Время: '+seconds(summary.get('elapsed_s'))]
        if not row.get('ok') and row.get('reason'):fields.append('Код: '+row['reason'])
        lines=block('[РАСЧЁТ] '+operation,*fields)
        counts=summary.get('filter_counts',{})
        if 'decoded' in counts:
            lines+=block('[ОТБОР]',
                'GraspNet: '+str(counts.get('decoded')),
                'После геометрических фильтров: '+str(counts.get('after_depth')),
                'После привязки к предмету: '+str(counts.get('after_target_binding','—')),
                'После NMS: '+str(counts.get('after_nms','—')),
                'вариантов после IK: '+str(counts.get('ik_feasible','—')))
        binding=summary.get('target_binding',{})
        if binding.get('enabled'):
            rows=binding.get('candidates',[])
            lines+=block('[ПРИВЯЗКА К ПРЕДМЕТУ]',
                'Недостаточно поверхности предмета: '+str(sum(r.get('reason')=='INSUFFICIENT_TARGET' for r in rows)),
                'Недостаточная доля предмета: '+str(sum(r.get('reason')=='LOW_TARGET_PURITY' for r in rows)))
        return lines
    return []
